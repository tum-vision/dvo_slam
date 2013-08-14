all: sophus

#	GIT_DIR: name of directory into which you want to clone to 
GIT_DIR = g2o_src
# GIT_URL: full URL to download
GIT_URL=git://github.com/RainerKuemmerle/g2o.git
#	GIT_PATCH: your (list of) patch file(s) to patch the downloaded software
#	GIT_REVISION:  -

include $(shell rospack find mk)/git_checkout.mk

sophus: download
	mkdir -p build && cd build && \
	cmake -DCMAKE_INSTALL_PREFIX=`pwd`/..  ../$(GIT_DIR) && \
  make install

clean: 
	rm -rf $(GIT_DIR) patched build include lib rospack_nosubdirs

