/**
 *  This file is part of dvo.
 *
 *  Copyright 2013 Christian Kerl <christian.kerl@in.tum.de> (Technical University of Munich)
 *  For more information see <http://vision.in.tum.de/data/software/dvo>.
 *
 *  dvo is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  dvo is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with dvo.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <dvo_slam/visualization/graph_visualizer.h>
#include <dvo_slam/GraphVisualizerConfig.h>

#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>

#include <visualization_msgs/InteractiveMarker.h>
#include <visualization_msgs/InteractiveMarkerControl.h>

#include <dynamic_reconfigure/server.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <g2o/types/slam3d/vertex_se3.h>
#include <g2o/types/slam3d/edge_se3.h>

namespace dvo_slam
{
namespace visualization
{

namespace internal
{

struct CompareEdgeChi2
{
  bool operator() (g2o::OptimizableGraph::Edge* e1, g2o::OptimizableGraph::Edge* e2)
  {
    return e1->chi2() < e2->chi2();
  }
};

struct FindEdgeById
{
public:
  FindEdgeById(int id) : id_(id)
  {

  }

  bool operator() (g2o::HyperGraph::Edge* e)
  {
    return e->id() == id_;
  }
private:
  int id_;
};

class GraphVisualizerImpl
{
private:
  ros::NodeHandle nh_, nh_graph_;
  dvo_ros::visualization::RosCameraTrajectoryVisualizer& visualizer_;
  interactive_markers::InteractiveMarkerServer* marker_server_;
  interactive_markers::MenuHandler menu_handler_;
  dynamic_reconfigure::Server<dvo_slam::GraphVisualizerConfig> reconfigure_server_;
  cv::Mat loop_closure_colors_;

  image_transport::ImageTransport it_;
  image_transport::Publisher image_topic_;

  dvo_slam::KeyframeGraph* graph_;

  double visible_;
  bool editable_;

  std::string edge_id_prefix_;
  std::set<std::string> editable_edges_;

public:
  friend class ::dvo_slam::visualization::GraphVisualizer;

  GraphVisualizerImpl(dvo_ros::visualization::RosCameraTrajectoryVisualizer& visualizer) :
    nh_("dvo_vis"),
    nh_graph_(nh_, "graph"),
    visualizer_(visualizer),
    marker_server_(0),
    menu_handler_(),
    reconfigure_server_(nh_graph_),
    it_(nh_),
    graph_(0),
    edge_id_prefix_("edge_"),
    keyframe1(-1)
  {
    void* native_visualizer;
    if(visualizer_.native(native_visualizer))
    {
      marker_server_ = reinterpret_cast<interactive_markers::InteractiveMarkerServer *>(native_visualizer);
      menu_handler_.insert("error with measurement", boost::bind(&GraphVisualizerImpl::onEdgeErrorWithMeasurementClick, this, _1));
      menu_handler_.insert("error with relative", boost::bind(&GraphVisualizerImpl::onEdgeErrorWithRelativeClick, this, _1));
      menu_handler_.insert("delete", boost::bind(&GraphVisualizerImpl::onEdgeDeleteClick, this, _1));
    }
    image_topic_ = it_.advertise("image", 1, true);

    allocateColors(10, 0.0f, 120.0f, loop_closure_colors_);

    reconfigure_server_.setCallback(boost::bind(&GraphVisualizerImpl::onConfig, this, _1, _2));
  }

  ~GraphVisualizerImpl()
  {
  }

  void onConfig(dvo_slam::GraphVisualizerConfig& cfg, uint32_t level)
  {
    visible_ = cfg.visible;
    editable_ = cfg.editable;

    update();

    if(cfg.graph_opt_final)
    {
      cfg.graph_opt_final = false;

      if(graph_ != 0)
        graph_->finalOptimization();
    }

  }

  void onEdgeDeleteClick(const interactive_markers::MenuHandler::FeedbackConstPtr& feedback)
  {
    int edge_id;

    std::istringstream iss(feedback->marker_name);
    iss.ignore(edge_id_prefix_.size());
    iss >> edge_id;

    g2o::OptimizableGraph::EdgeSet::iterator edge_it = std::find_if(graph_->graph().edges().begin(), graph_->graph().edges().end(), FindEdgeById(edge_id));

    if(edge_it == graph_->graph().edges().end()) return;

    g2o::HyperGraph::Edge* edge = *edge_it;
  }

  void onEdgeErrorWithMeasurementClick(const interactive_markers::MenuHandler::FeedbackConstPtr& feedback)
  {
    cv::Mat error = graph_->computeIntensityErrorImage(parseEdgeIdFromMarker(feedback->marker_name), true);

    if(error.total() > 0)
      publishErrorImage(error);
  }

  void onEdgeErrorWithRelativeClick(const interactive_markers::MenuHandler::FeedbackConstPtr& feedback)
  {
    cv::Mat error = graph_->computeIntensityErrorImage(parseEdgeIdFromMarker(feedback->marker_name), false);

    if(error.total() > 0)
      publishErrorImage(error);
  }

  void publishErrorImage(const cv::Mat& img)
  {
    std_msgs::Header h;

    cv::Mat error, error_uc8;
    error = cv::abs(img * 255.0f);
    error.convertTo(error_uc8, CV_8UC1);

    cv_bridge::CvImage ros_img(h, "mono8", error_uc8);
    image_topic_.publish(ros_img.toImageMsg());
  }

  int parseEdgeIdFromMarker(const std::string& marker_name)
  {
    int edge_id;

    std::istringstream iss(marker_name);
    iss.ignore(edge_id_prefix_.size());
    iss >> edge_id;

    return edge_id;
  }

  geometry_msgs::Point toPoint(g2o::VertexSE3* v)
  {
    geometry_msgs::Point p;

    p.x = v->estimate().translation()(0);
    p.y = v->estimate().translation()(1);
    p.z = v->estimate().translation()(2);

    return p;
  }

  void allocateColors(size_t n, float min_hue, float max_hue, cv::Mat& rgb)
  {
    cv::Mat_<cv::Vec3f> hsv;
    hsv.create(n, 1);
    for(size_t idx = 0; idx < static_cast<size_t>(hsv.rows); ++idx)
    {
      cv::Vec3f& c = hsv.at<cv::Vec3f>(idx);
      c.val[0] = (max_hue - min_hue) / float(hsv.rows) * float(idx);
      c.val[1] = 1.0f;
      c.val[2] = 1.0f;
    }

    cv::cvtColor(hsv, rgb, CV_HSV2RGB);
  }

  int keyframe1;

  void onKeyframeClick(short keyframe_id, const dvo::visualization::CameraVisualizer&)
  {
    bool has_first = keyframe1 != -1;

    if(has_first)
    {
      if(keyframe_id != keyframe1)
        graph_->debugLoopClosureConstraint(keyframe1, keyframe_id);

      keyframe1 = -1;
    }
    else
    {
      keyframe1 = keyframe_id;
    }
  }

  void update()
  {
    if(marker_server_ == 0 || graph_ == 0) return;

    for(KeyframeVector::const_iterator it = graph_->keyframes().begin(); it != graph_->keyframes().end(); ++it)
    {
      const KeyframePtr& keyframe = *it;

      std::stringstream id;
      id << "keyframe_" << keyframe->id();

      visualizer_.camera(id.str())
          ->color(dvo::visualization::Color::blue())
          .onclick(boost::bind(&GraphVisualizerImpl::onKeyframeClick, this, keyframe->id(), _1))
          .update(keyframe->image()->level(0), keyframe->pose())
          .show(dvo::visualization::CameraVisualizer::ShowCameraAndCloud);

    }

      visualization_msgs::Marker m_odometry, m_loop;
      m_odometry.type = visualization_msgs::Marker::LINE_LIST;
      m_odometry.color.a = 1.0f;
      m_odometry.color.r = 1.0f;
      m_odometry.color.g = 0.0f;
      m_odometry.color.b = 1.0f;
      m_odometry.scale.x = 0.005;

      m_loop = m_odometry;
      m_loop.color.r = 0.0f;
      m_loop.color.g = 1.0f;
      m_loop.color.b = 1.0f;
      m_loop.scale.x = 0.01;

      std_msgs::ColorRGBA loop_color;
      loop_color.r = loop_color.g = loop_color.b = loop_color.a = 1.0;

      typedef std::vector<g2o::OptimizableGraph::Edge*> EdgeVector;
      EdgeVector loop_closure_edges;

      for(g2o::HyperGraph::EdgeSet::const_iterator it = graph_->graph().edges().begin(); it !=  graph_->graph().edges().end(); ++it)
      {
        g2o::OptimizableGraph::Edge* edge = (g2o::OptimizableGraph::Edge*)(*it);

        // skip if just prior edge
        if(edge->vertices().size() < 2) continue;

        g2o::VertexSE3* v1 = (g2o::VertexSE3*)edge->vertex(0);
        g2o::VertexSE3* v2 = (g2o::VertexSE3*)edge->vertex(1);

        //if((v1->id() * v2->id()) < 0) continue;

        if(edge->vertices().size() == 2 && edge->level() == 0)
        {
          int dist = std::abs(edge->vertex(0)->id() - edge->vertex(1)->id());

          if(dist == 1)
          {
            if(edge->vertex(0)->id() > 0 || edge->vertex(1)->id() > 0)
            {
              m_odometry.points.push_back(toPoint(v1));
              m_odometry.points.push_back(toPoint(v2));
            }
          }
          else
          {
            if(edge->vertex(0)->id() > 0 && edge->vertex(1)->id() > 0)
            {
              edge->computeError();
              loop_closure_edges.push_back(edge);
            }
          }
        }
      }

      std::set<std::string> previous_editable_edges = editable_edges_;
      editable_edges_.clear();

      if(!loop_closure_edges.empty())
      {

        std::sort(loop_closure_edges.begin(), loop_closure_edges.end(), CompareEdgeChi2());

        float chi2_min = loop_closure_edges.front()->chi2(), chi2_max = loop_closure_edges.back()->chi2();
        float chi2_normalizer = chi2_max - chi2_min;

        size_t offset = size_t((1 - visible_) * loop_closure_edges.size());

        for(EdgeVector::iterator it = loop_closure_edges.begin() + offset; it != loop_closure_edges.end(); ++it)
        {
          int c_idx = std::min(std::max(loop_closure_colors_.rows - 1 - int(((*it)->chi2() - chi2_min) / chi2_normalizer * (loop_closure_colors_.rows - 1)), 0), loop_closure_colors_.rows - 1);

          cv::Vec3f& c = loop_closure_colors_.at<cv::Vec3f>(c_idx);

          //std::cerr << c_idx << " " << idx  << " " << m_loop.colors.size() << " "  << chi2_min  << " " << chi2_max  << " " << m_loop.colors[idx].a << std::endl;

          std_msgs::ColorRGBA rgba;
          rgba.r = c.val[0]; rgba.g = c.val[1]; rgba.b = c.val[2]; rgba.a = 1.0;

          g2o::VertexSE3 *v0 = dynamic_cast<g2o::VertexSE3*>((*it)->vertex(0)), *v1 = dynamic_cast<g2o::VertexSE3*>((*it)->vertex(1));

          m_loop.points.push_back(toPoint(v0));
          m_loop.points.push_back(toPoint(v1));

          m_loop.colors.push_back(rgba);
          m_loop.colors.push_back(rgba);

          if(editable_)
          {
            Eigen::Vector3d p = 0.5 * (v0->estimate().translation() + v1->estimate().translation());

            visualization_msgs::Marker m_box;
            m_box.type = visualization_msgs::Marker::CUBE;
            m_box.color = rgba;
            m_box.scale.x = 0.02f; m_box.scale.y = 0.02f; m_box.scale.z = 0.02f;

            visualization_msgs::InteractiveMarkerControl m_control;
            m_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;
            m_control.always_visible = true;
            m_control.markers.push_back(m_box);

            std::stringstream edge_id_builder;
            edge_id_builder << edge_id_prefix_ << (*it)->id();
            std::string edge_id = edge_id_builder.str();

            visualization_msgs::InteractiveMarker m;
            m.header.frame_id = "/world";
            m.name = edge_id;
            m.pose.position.x = p(0);
            m.pose.position.y = p(1);
            m.pose.position.z = p(2);
            m.controls.push_back(m_control);

            marker_server_->insert(m, boost::bind(&GraphVisualizerImpl::onEdgeErrorWithMeasurementClick, this, _1));
            editable_edges_.insert(edge_id);
            previous_editable_edges.erase(edge_id);
          }
        }
      }

      for(std::set<std::string>::iterator it = previous_editable_edges.begin(); it != previous_editable_edges.end(); ++it)
      {
        marker_server_->erase(*it);
      }

      for(std::set<std::string>::iterator it = editable_edges_.begin(); it != editable_edges_.end(); ++it)
      {
        menu_handler_.apply(*marker_server_, *it);
      }

      visualization_msgs::InteractiveMarkerControl control;
      control.always_visible = true;
      control.markers.push_back(m_odometry);
      control.markers.push_back(m_loop);

      visualization_msgs::InteractiveMarker marker;

      marker.header.frame_id = "/world";
      marker.name = std::string("constraints");
      marker.controls.push_back(control);

      marker_server_->insert(marker);
      marker_server_->applyChanges();

  }
};

} /* namespace internal */

GraphVisualizer::GraphVisualizer(dvo_ros::visualization::RosCameraTrajectoryVisualizer& visualizer) :
    impl_(new internal::GraphVisualizerImpl(visualizer))
{
}

GraphVisualizer::~GraphVisualizer()
{
}

void GraphVisualizer::setGraph(dvo_slam::KeyframeGraph* graph)
{
  impl_->graph_ = graph;
}

void GraphVisualizer::update()
{
  impl_->update();
}

} /* namespace visualization */
} /* namespace dvo_slam */
