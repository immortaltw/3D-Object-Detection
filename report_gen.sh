#!/bin/bash

# Reset
rm -rf report *csv

# Build
rm -rf build && mkdir build && cd build && cmake ../ && make

# Hyper params
DETECTOR_TYPES="HARRIS SHITOMASI FAST BRISK ORB"
DESCRIPTOR_TYPES="BRISK BRIEF ORB FREAK"
MATCHER_TYPE="MAT_BF"   # MAT_BF MAT_FLANN
DESCRIPTOR_CAT="DES_BINARY" # DES_BINARY DES_HOG
SELECTOR_TYPE="SEL_KNN"  # SEL_NN SEL_KNN

for DET in $DETECTOR_TYPES
do
    for DEC in $DESCRIPTOR_TYPES
    do
        ./3D_object_tracking $DET $DEC $MATCHER_TYPE DES_BINARY $SELECTOR_TYPE
    done
done

# SIFT
# SIFT (L2_NORM)
DESCRIPTOR_TYPES="BRISK BRIEF FREAK SIFT"
for DEC in $DESCRIPTOR_TYPES
do
    ./3D_object_tracking SIFT $DEC $MATCHER_TYPE DES_HOG $SELECTOR_TYPE
done

# AKAZE
# AKAZE BRISK BRIEF FREAK SIFT
DESCRIPTOR_TYPES="AKAZE BRISK BRIEF FREAK SIFT"
for DEC in $DESCRIPTOR_TYPES
do
    ./3D_object_tracking AKAZE $DEC $MATCHER_TYPE DES_HOG $SELECTOR_TYPE
done

cd ../
mkdir report
mv *.csv report/

which conda
if [[ "$?" == 0 ]]; then
    conda install -y pandas
else 
    pip3 install pandas
fi
python3 report_parser.py