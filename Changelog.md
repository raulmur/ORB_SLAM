#Changelog of ORB-SLAM Monocular

## 1.0.1 (18-01-2016)
- Save/load vocabulary from text file instead of cv::FileStorage. 
- Unused files from Thirdparty/DBoW2 and Thirdparty/g2o have been removed. g2o files have been reorganized and we compile only
one library libg2o.so instead of four libraries (libg2o_core.so, libg2o_types.so, etc)
- We use now Eigen solver instead of Cholmod solver in g2o. Removed dependency on Suitesparse and Cholmod.
- Several bugs have been fixed.

## 1.0.0 (03-03-2015)
First ORB-SLAM Monocular version
