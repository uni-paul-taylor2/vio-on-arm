#!/bin/bash

: '
In order for this script to work it needs ROS and TagSLAM to be installed.
Follow these instrunctions to install TagSLAM: https://github.com/berndpfrommer/tagslam_root?tab=readme-ov-file
This script was created following these instructions: https://berndpfrommer.github.io/tagslam_web/making_tags/#how-to-print-your-own-apriltags

Note: borderbits refer to a border of solid black bits that surround the tag (defualts to 2),
		however each tag also needs to be surrounded by a white border (outside the black border) approx. 2 bits wide.

Printing settings: Paper Size: Letter, Layout: Landscape, Scale: Actual Size

By defualt the tags would be printed in the centre of the page leaving a sufficiently large white border around them without having to explicitly create one.
 '

size=0.15 #size in metres
boarder=1 #num of boarder bits (solid black border around tag)

#gen first 30 36h11 and 16h5 tags
for i in {0..29}
do
	rosrun tagslam make_tag.py ./36h11Tags/36h11_$i --nx 1 --ny 1 --marginx 0.00 --marginy 0.00 --tsize $size --tspace 0.0 --startid $i --tfam t36h11 --borderbits $boarder
	rosrun tagslam make_tag.py ./16h5Tags/16h5_$i --nx 1 --ny 1 --marginx 0.00 --marginy 0.00 --tsize $size --tspace 0.0 --startid $i --tfam t16h5 --borderbits $boarder
done
