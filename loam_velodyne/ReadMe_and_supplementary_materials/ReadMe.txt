This package is NOT the original package implemented by Ji Zhang.
The original files from Ji Zhang can be found on ROS package documentation archive (http://docs.ros.org/indigo/api/loam_velodyne/html/files.html). Also attached in this folder (old_src) folder.
some example videos can be found:
https://www.youtube.com/watch?v=8ezyhTAEyHs#t=155
https://www.youtube.com/watch?v=ld2lO5mnjO8
https://plus.google.com/+JiZhang_CMU/posts

This package is the modified version of the original by laboshinl. 
(https://github.com/laboshinl)
It doesnot work well with "nsh_indoor_outdoor.bag" data or "play gates_oscillating_motion.bag" data. The main change by laboshinl is
--
I've created package and launch file. I've also modified scanRegistration.cpp to get it to work with my VLP16 (I think originally it was for HDL32 )
--



