#  Software License Agreement (BSD License)
#  Copyright (c) 2003-2023, CHAI3D
#  (www.chai3d.org)
#
#  All rights reserved.
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions
#  are met:
#
#  * Redistributions of source code must retain the above copyright
#  notice, this list of conditions and the following disclaimer.
#
#  * Redistributions in binary form must reproduce the above
#  copyright notice, this list of conditions and the following
#  disclaimer in the documentation and/or other materials provided
#  with the distribution.
#
#  * Neither the name of CHAI3D nor the names of its contributors may
#  be used to endorse or promote products derived from this software
#  without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#  POSSIBILITY OF SUCH DAMAGE.


# backups
cp $1 chai3d-master.png

# create ICNS
mkdir chai3d.iconset
sips -z 16 16     $1 --out chai3d.iconset/icon_16x16.png
sips -z 32 32     $1 --out chai3d.iconset/icon_16x16@2x.png
sips -z 32 32     $1 --out chai3d.iconset/icon_32x32.png
sips -z 64 64     $1 --out chai3d.iconset/icon_32x32@2x.png
sips -z 128 128   $1 --out chai3d.iconset/icon_128x128.png
sips -z 256 256   $1 --out chai3d.iconset/icon_128x128@2x.png
sips -z 256 256   $1 --out chai3d.iconset/icon_256x256.png
sips -z 512 512   $1 --out chai3d.iconset/icon_256x256@2x.png
sips -z 512 512   $1 --out chai3d.iconset/icon_512x512.png
sips -z 1024 1024 $1 --out chai3d.iconset/icon_512x512@2x.png
iconutil -c icns -o chai3d.icns chai3d.iconset

# create RSRC
sips -i chai3d.icns
DeRez -only icns chai3d.icns > chai3d.rsrc

# create ICO
sips -z 256 256 $1 --out chai3d_256x256.png
sips -z 128 128 $1 --out chai3d_128x128.png
sips -z 64 64   $1 --out chai3d_64x64.png
sips -z 48 48   $1 --out chai3d_48x48.png
sips -z 32 32   $1 --out chai3d_32x32.png
sips -z 16 16   $1 --out chai3d_16x16.png
icotool -c chai3d_256x256.png chai3d_128x128.png chai3d_64x64.png chai3d_48x48.png chai3d_32x32.png chai3d_16x16.png > chai3d.ico

# create favicon
icotool -c chai3d_16x16.png > favicon.ico

# create PNG and logo
sips -z 512 512 $1 --out chai3d.png
sips -z 22 22   $1 --out logo-icon.png

# cleanup
rm -R chai3d.iconset
rm chai3d_*x*.png
