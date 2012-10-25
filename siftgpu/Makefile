all: siftgpu


#TARBALL = build/siftgpu.zip
#TARBALL_URL = http://wwwx.cs.unc.edu/~ccwu/cgi-bin/siftgpu.cgi

#The following tarball has been generated as follows:
#  cd build/SiftGPU
#  rm -rf bin
#  rm -rf build
#  cd ..
#  zip siftgpu.zip SiftGPU -r
#  mv siftgpu.zip ..
#
#  once compilation warnings have been fixed in build/SiftGPU
#
TARBALL = siftgpu.zip
SOURCE_DIR = build/SiftGPU
MD5SUM_FILE = siftgpu.zip.md5sum
UNPACK_CMD = unzip -o

INSTALL_DIR = lib
INCLUDE_DIR = include

include $(shell rospack find mk)/download_unpack_build.mk

siftgpu: $(INSTALL_DIR)/installed

$(INSTALL_DIR)/installed: $(SOURCE_DIR)/unpacked
	mkdir -p $(INSTALL_DIR)
	mkdir -p $(INCLUDE_DIR)
	cp alt_makefile_inside $(SOURCE_DIR)/makefile
	cd $(SOURCE_DIR)    \
	&& make 
	cp $(SOURCE_DIR)/bin/libsiftgpu.so $(INSTALL_DIR)/ 
	cp $(SOURCE_DIR)/src/SiftGPU/*.h $(INCLUDE_DIR)/
#	rm -rf src
	touch $(INSTALL_DIR)/installed
clean:
	rm -rf src $(SOURCE_DIR) $(INSTALL_DIR) $(INCLUDE_DIR)

wipe: clean
	rm -rf build
