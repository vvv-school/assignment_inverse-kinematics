#!/bin/bash

# Copyright: (C) 2016 iCub Facility - Istituto Italiano di Tecnologia
# Authors: Ugo Pattacini <ugo.pattacini@iit.it>
# CopyPolicy: Released under the terms of the GNU GPL v3.0.

get_helpers=false
branch=master
if [ $# -gt 0 ]; then
    if [ "$1" == "--get-helpers" ]; then
        get_helpers=true
        if [ $# -gt 1 ]; then
            branch=$2
        fi
    elif [ "$1" == "--help" ]; then
        echo ""
        echo "Usage: $0 [--get-helpers [branch|tag]]"
        echo "\"--get-helpers [branch|tag]\" forces downloading helper tools anew on specified branch or tag (default = master)"
        echo ""
        exit 0
    fi
fi

# color codes
red='\033[1;31m'
green='\033[1;32m'
nc='\033[0m'

code_dir=$(pwd)/../
test_dir=$(pwd)

if [ "$get_helpers" == "true" ] && [ -d build ]; then
    rm -Rf build
fi

if [ -d build ]; then
    cd build
else
    mkdir build && cd build
    git clone https://github.com/vvv-school/vvv-school.github.io.git --depth 1 --branch $branch helpers
fi
build_dir=$(pwd)

if [ $? -eq 0 ]; then
    if [ -f "${test_dir}"/test-type ]; then
        test_type=$(head -1 "${test_dir}"/test-type)
        ./helpers/scripts/smoke-test-${test_type}.sh "$build_dir" "$code_dir" "$test_dir"
        ret=$?
    else
        echo -e "${red}test-type is missing!${nc}"
        ret=252
    fi
else
    echo -e "${red}GitHub seems unreachable${nc}"
    ret=252
fi

cd ../
exit $ret
