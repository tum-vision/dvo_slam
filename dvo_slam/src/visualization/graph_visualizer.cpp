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

#include <interactive_markers/interactive_marker_server.h>

#include <g2o/types/slam3d/vertex_se3.h>
#include <g2o/types/slam3d/edge_se3.h>

namespace dvo_slam
{
namespace visualization
{

namespace internal
{

class GraphVisualizerImpl
{
private:
  dvo_ros::visualization::RosCameraTrajectoryVisualizer& visualizer_;
  interactive_markers::InteractiveMarkerServer* marker_server_;

public:
  GraphVisualizerImpl(dvo_ros::visualization::RosCameraTrajectoryVisualizer& visualizer) :
    visualizer_(visualizer),
    marker_server_(0)
  {
    void* native_visualizer;
    if(visualizer_.native(native_visualizer))
    {
      marker_server_ = reinterpret_cast<interactive_markers::InteractiveMarkerServer *>(native_visualizer);
    }
  }

  ~GraphVisualizerImpl()
  {
  }

  void visualize(const dvo_slam::KeyframeGraph& map)
  {
    if(marker_server_ == 0) return;

    for(KeyframeVector::const_iterator it = map.keyframes().begin(); it != map.keyframes().end(); ++it)
    {
      const KeyframePtr& keyframe = *it;

      std::stringstream id;
      id << "keyframe_" << keyframe->id();

      visualizer_.camera(id.str())
          ->color(dvo::visualization::Color::blue())
          .update(keyframe->image()->level(0), keyframe->pose())
          .show(dvo::visualization::CameraVisualizer::ShowCameraAndCloud);
    }

      /*
      if(false && updated_system)
      {
        visualization_msgs::InteractiveMarkerControl control_cov;
        control_cov.always_visible = true;

        for(KeyframeVector::const_iterator it = keyframes_.begin(); it != keyframes_.end(); ++it)
        {
          const KeyframePtr& keyframe = *it;

          std::stringstream id;
          id << "keyframe_" << keyframe->id();

          //marker_server->setCallback(id.str(), boost::bind(&KeyframeGraphImpl::onMarkerFeedback, this, _1, keyframe->id()));

          g2o::VertexSE3 *kv = (g2o::VertexSE3*) keyframegraph_.vertex(keyframe->id());
          assert(kv != 0);

          if(kv->A().data() == 0) continue;

          Eigen::Matrix3d cov = kv->A().inverse().block<3, 3>(3, 3);
          Eigen::EigenSolver<Eigen::Matrix3d> es(cov);

          if(es.info() == Eigen::Success)
          {
            Eigen::Quaterniond q(es.eigenvectors().real());

            visualization_msgs::Marker m_cov;
            m_cov.type = visualization_msgs::Marker::SPHERE;
            m_cov.id = keyframe->id();
            m_cov.color.a = 0.5f;
            m_cov.color.r = 1.0f;
            m_cov.color.g = 1.0f;
            m_cov.color.b = 1.0f;
            m_cov.scale.x = es.eigenvalues().real()(0) * 5e5;
            m_cov.scale.y = es.eigenvalues().real()(1) * 5e5;
            m_cov.scale.z = es.eigenvalues().real()(2) * 5e5;
            m_cov.pose.position.x = kv->estimate().translation()(0);
            m_cov.pose.position.y = kv->estimate().translation()(1);
            m_cov.pose.position.z = kv->estimate().translation()(2);
            m_cov.pose.orientation.x = q.x();
            m_cov.pose.orientation.y = q.y();
            m_cov.pose.orientation.z = q.z();
            m_cov.pose.orientation.w = q.w();

            control_cov.markers.push_back(m_cov);
          }
        }

        visualization_msgs::InteractiveMarker marker_cov;

        marker_cov.header.frame_id = "/world";
        marker_cov.name = std::string("covariance");
        marker_cov.controls.push_back(control_cov);

        marker_server->insert(marker_cov);
        marker_server->applyChanges();
      }
      */
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

      float chi2_min = std::numeric_limits<float>::max(), chi2_max = std::numeric_limits<float>::min();

      for(g2o::HyperGraph::EdgeSet::const_iterator it = map.graph().edges().begin(); it != map.graph().edges().end(); ++it)
      {
        g2o::OptimizableGraph::Edge* edge = (g2o::OptimizableGraph::Edge*)(*it);

        // skip if just prior edge
        if(edge->vertices().size() < 2) continue;

        std::stringstream id;
        id << "edge_" << edge->id();

        geometry_msgs::Point p1, p2;
        g2o::VertexSE3* v1 = (g2o::VertexSE3*)edge->vertex(0);
        g2o::VertexSE3* v2 = (g2o::VertexSE3*)edge->vertex(1);

        //if((v1->id() * v2->id()) < 0) continue;


        p1.x = v1->estimate().translation()(0);
        p1.y = v1->estimate().translation()(1);
        p1.z = v1->estimate().translation()(2);
        p2.x = v2->estimate().translation()(0);
        p2.y = v2->estimate().translation()(1);
        p2.z = v2->estimate().translation()(2);

        if(edge->vertices().size() == 2 && edge->level() == 0)
        {
          int dist = std::abs(edge->vertex(0)->id() - edge->vertex(1)->id());

          if(dist == 1)
          {
            //if(edge->vertex(0)->id() < 0 || edge->vertex(1)->id() < 0)
            {
              m_odometry.points.push_back(p1);
              m_odometry.points.push_back(p2);
            }
          }
          else
          {
            m_loop.points.push_back(p1);
            m_loop.points.push_back(p2);
            edge->computeError();
            float e(edge->chi2());

            chi2_min = std::min(chi2_min, e);
            chi2_max = std::max(chi2_max, e);

            loop_color.a = e;

            m_loop.colors.push_back(loop_color);
            m_loop.colors.push_back(loop_color);
          }
        }
      }

      cv::Mat_<cv::Vec3f> hsv, rgb;
      hsv.create(100, 1);
      float min_h = 0.0f, max_h = 120.0f;
      for(size_t idx = 0; idx < hsv.rows; ++idx)
      {
        cv::Vec3f& c = hsv.at<cv::Vec3f>(idx);
        c.val[0] = (max_h - min_h) / float(hsv.rows) * float(idx);
        c.val[1] = 1.0f;
        c.val[2] = 1.0f;
      }

      cv::cvtColor(hsv, rgb, CV_HSV2RGB);

      float chi2_normalizer = chi2_max - chi2_min;

      for(size_t idx = 0; idx < m_loop.colors.size(); idx += 2)
      {
        int c_idx = std::min(std::max(99 - int((m_loop.colors[idx].a - chi2_min) / chi2_normalizer * 99), 0), 99);

        cv::Vec3f& c = rgb.at<cv::Vec3f>(c_idx);

        //std::cerr << c_idx << " " << idx  << " " << m_loop.colors.size() << " "  << chi2_min  << " " << chi2_max  << " " << m_loop.colors[idx].a << std::endl;

        m_loop.colors[idx + 0].r = m_loop.colors[idx + 1].r = c.val[0];
        m_loop.colors[idx + 0].g = m_loop.colors[idx + 1].g = c.val[1];
        m_loop.colors[idx + 0].b = m_loop.colors[idx + 1].b = c.val[2];
        m_loop.colors[idx + 0].a = m_loop.colors[idx + 1].a = 0.0;
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


void GraphVisualizer::visualize(const dvo_slam::KeyframeGraph& map)
{
  impl_->visualize(map);
}

} /* namespace visualization */
} /* namespace dvo_slam */
