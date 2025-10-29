#! /bin/bash

#  Software License Agreement (BSD License)
#  Copyright (c) 2003-2023, CHAI3D
#  (www.chai3d.org)
#
#  All rights reserved.
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions
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

# This script builds versioning info and applies it to various use cases:
# case 0 - compose version string (-PdMmprefv)
# case 1 - target is a header file, update or generate
# case 2 - no target is given, create versioned <file> for each <file>.ver
# case 3 - target is a file, version all fields
# case 4 - target is a folder, recursively version all files within

# This function replaces version fields with values in filename argument
function version {
    file=$1
    if [ -f $file ]; then
        LC_ALL=C sed -i.sed-unversioned -e 's/$PROJECT/'$PROJECT'/g; s/$PREFIX/'$PREFIX'/g; s/$DESCRIPTION/'"$DESCRIPTION"'/g; s/$VERSION_FULL/'$VERSION_FULL'/g; s/$VERSION/'$VERSION'/g; s/$MAJOR/'$MAJOR'/g; s/$MINOR/'$MINOR'/g; s/$PATCH/'$PATCH'/g; s/$PRERELEASE/'$PRERELEASE'/g; s/$METADATA/'$METADATA'/g; s/$REVISION/'$REVISION'/g' $file
        LC_ALL=C rm -f $file.sed-unversioned
    fi
}

# retrieve arguments
invocationDir=`pwd`
scriptDir=$( cd "$(dirname "${BASH_SOURCE[0]}")" ; pwd -P )
versionFile=$1
target=$2

# if version file does not exist, stop
if [ -f "$versionFile" ]; then
    :
else
    echo "$versionFile not found"
    exit -1
fi

# retrieve version info
PROJECT=`cat $versionFile | grep PROJECT | awk 'BEGIN {FS="="} {printf $2}'`
PREFIX=`cat $versionFile | grep PREFIX | awk 'BEGIN {FS="="} {printf $2}'`
DESCRIPTION=`cat $versionFile | grep DESCRIPTION | awk 'BEGIN {FS="="} {printf $2}'`
MAJOR=`cat $versionFile | grep MAJOR | awk 'BEGIN {FS="="} {printf $2}'`
MINOR=`cat $versionFile | grep MINOR | awk 'BEGIN {FS="="} {printf $2}'`
PATCH=`cat $versionFile | grep PATCH | awk 'BEGIN {FS="="} {printf $2}'`
PRERELEASE=`cat $versionFile | grep PRERELEASE | awk 'BEGIN {FS="="} {printf $2}'`
METADATA=`cat $versionFile | grep "METADATA=" | awk 'BEGIN {FS="="} {print $2}'`
REVISION=`cat $versionFile | grep "REVISION=" | awk 'BEGIN {FS="="} {print $2}'`
if [ -z "$PREFIX" ]; then
    PREFIX=`echo $PROJECT | tr a-z A-Z`_
fi
if [ -z "$METADATA" ]; then
    METADATA=+g`git describe --always --dirty --exclude "*"`
    if [ $? -ne 0 ]; then
        METADATA=
    fi
fi
if [ -z "$REVISION" ]; then
    REVISION=0x`git describe --always --exclude "*"`
    if [ $? -ne 0 ]; then
        REVISION=0x0000000
    fi
fi
VERSION_FULL=$MAJOR.$MINOR.$PATCH$PRERELEASE$METADATA
if [ -z "$PRERELEASE" ]; then
    VERSION=$MAJOR.$MINOR.$PATCH
else
    VERSION=$VERSION_FULL
fi

# case 0 - composition
if [ "${target:0:1}" == "-" ]; then
    for (( i=1; i<${#target}; i++ )); do
        case "${target:$i:1}" in
        "P")
            echo -n "$PROJECT"
            ;;
        "d")
            echo -n "$DESCRIPTION"
            ;;
        "M")
            echo -n "$MAJOR"
            ;;
        "m")
            echo -n "$MINOR"
            ;;
        "p")
            echo -n "$PATCH"
            ;;
        "r")
            echo -n "$PRERELEASE"
            ;;
        "e")
            echo -n "$METADATA"
            ;;
        "f")
            echo "$VERSION_FULL"
            exit 0
            ;;
        "v")
            echo "$VERSION"
            exit 0
            ;;
        *)
            echo -n "${target:$i:1}"
        esac
    done
    echo
    exit 0
fi

echo "versioning $PROJECT $VERSION"

# case 1 - target is a header file, update or generate
if [ "${target:(-2)}" == ".h" ]; then
    targetFolder=$(dirname -- "$target")
    targetFilename=$(basename -- "$target")
    targetExtension="${targetFilename##*.}"
    targetFilename="${targetFilename%.*}"
    if [ ! -d "$targetFolder" ]; then
        mkdir -p $targetFolder
    fi
    if [ -f "$target" ]; then
        _VERSION_FULL=`cat $target | grep "VERSION_FULL" | awk '{print $3}'`
        if [ "$_VERSION_FULL" == "\"$VERSION_FULL\"" ]; then
            generateFile=false
        else
            generateFile=true
        fi
    else
        generateFile=true
    fi
    if $generateFile; then
        echo "// version info"                                     > $target
        echo "#define ${PREFIX}DESCRIPTION     \"$DESCRIPTION\""  >> $target
        echo "#define ${PREFIX}VERSION_MAJOR   $MAJOR"            >> $target
        echo "#define ${PREFIX}VERSION_MINOR   $MINOR"            >> $target
        echo "#define ${PREFIX}VERSION_PATCH   $PATCH"            >> $target
        echo "#define ${PREFIX}PRERELEASE      \"$PRERELEASE\""   >> $target
        echo "#define ${PREFIX}METADATA        \"$METADATA\""     >> $target
        echo "#define ${PREFIX}VERSION         \"$VERSION\""      >> $target
        echo "#define ${PREFIX}VERSION_FULL    \"$VERSION_FULL\"" >> $target
        echo "#define ${PREFIX}REVISION        $REVISION"         >> $target
    fi
    exit 0
fi

# case 2 - no target is given, create versioned <file> for each <file>.ver
if [ -z "$target" ]
then
    filelist=`find $invocationDir -type f -name "*.ver"`
    for verfile in $filelist
    do
        file=${verfile%.ver}
        cp $verfile $file
        version $file
    done
fi

# case 3 - target is a file, version all fields
if [ -f "$target" ]
then
    version $target
fi

# case 4 - target is a folder, recursively version all files within
if [ -d "$target" ]
then
    filelist=`find $target -type f`
    for file in $filelist
    do
        version $file
    done
fi
