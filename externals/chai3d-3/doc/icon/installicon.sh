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

TOP_DIR=../..

# executables
list=". ./modules/GEL ./modules/ODE"
for module in $list
do
  mkdir -p $TOP_DIR/$module/bin/resources/icons
  cp chai3d.png  $TOP_DIR/$module/bin/resources/icons
  cp chai3d.ico  $TOP_DIR/$module/bin/resources/icons
  cp chai3d.icns $TOP_DIR/$module/bin/resources/icons
  cp chai3d.rsrc $TOP_DIR/$module/bin/resources/icons
  cp chai3d.rsrc $TOP_DIR/$module/bin/resources/icons
done

# documentation
list=". ./modules/GEL ./modules/ODE ./modules/V-REP"
for module in $list
do
  mkdir -p $TOP_DIR/$module/doc/resources
  cp logo-icon.png  $TOP_DIR/$module/doc/resources
  cp favicon.ico    $TOP_DIR/$module/doc/resources
done

# Qt
list="applications/Qt/Michelangelo examples/Qt/ModelViewer templates/Qt"
for qtapp in $list
do
  cp chai3d.ico $TOP_DIR/$qtapp
  cp chai3d.icns $TOP_DIR/$qtapp
done
