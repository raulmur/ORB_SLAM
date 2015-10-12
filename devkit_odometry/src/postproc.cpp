
#include "postproc.h"

#include <opencv2/core/core.hpp> //for InspectTsukubaEulerAngles()
#include <opencv2/highgui/highgui.hpp> //for InspectTsukubaEulerAngles()

#include <fstream>

using namespace std;
using namespace Eigen;
using namespace Sophus;
PostProcessor::PostProcessor(std::string input_dir, std::string output_dir):
    sInputPath(input_dir),sOutputPath(output_dir)
{
}
PostProcessor::~PostProcessor()
{
    Reset();
}
void PostProcessor::Reset()
{
    mRawPoses.clear(); mKeyPoses.clear();
    mReprojedPoses.clear(); mSpeedBias.clear();
}
void PostProcessor::Run(const std::vector<int>seqNum, bool bApplyReloc)
{
for (std::vector<int>::const_iterator it= seqNum.begin(), ite= seqNum.end(); it!=ite; ++it) {
    //output file name
    char file_name[256];
    sprintf(file_name,"%s/%02d.txt",sOutputPath.c_str(), *it);
    string output_file(file_name);
    sprintf(file_name,"%s/KITTISeq%02d.txt",sInputPath.c_str(), *it);
    string input_file(file_name);
    cerr<< "Processing seq "<< *it<<endl;
    LoadData(input_file);
    if(bApplyReloc)
        ProcessPoses();
    else
        mReprojedPoses=mRawPoses;
    SaveData(output_file);
}
}

// if apply relocation, all poses are transformed relative to their closest keyframe
void PostProcessor::Run(std::string sInputFilename, std::string sOutputFilename, bool bApplyReloc)
{
    //output file name
    char file_name[256];
    sprintf(file_name,"%s/%s",sOutputPath.c_str(), sOutputFilename.c_str());
    sOutputFilename=string(file_name);
    sprintf(file_name,"%s/%s",sInputPath.c_str(), sInputFilename.c_str());
    sInputFilename=string(file_name);
    LoadData(sInputFilename);
    if(bApplyReloc)
        ProcessPoses();
    else
        mReprojedPoses=mRawPoses;
    SaveData(sOutputFilename);
}
//in each data file, the first section are (time,pose(txyzinw, qc2w(xyzw)), speed, bias) of sequential frames, the second section are of keyframes
// note the first line of the first section is for the second frame, the pose of the first frame is identity
void PostProcessor::LoadData(const std::string sInputFile)
{
    Reset();
    ifstream stream(sInputFile.c_str());

    double last_time=-1;
    Eigen::Matrix<double,4,1> xyzw;
    bool first_section=true;
    const size_t nFields=17;
    double proxy[nFields]={0};

    string tempStr;
    getline(stream, tempStr); //remove first line
    mRawPoses.push_back(std::make_pair(0, SE3d()));//for the first frame
    while(!stream.eof()){
        for(size_t jack=0; jack<nFields; ++jack)
            stream>>proxy[jack];
        getline(stream, tempStr);
        if(stream.fail())
            break;
        Map<Matrix<double,nFields,1> > porter(proxy);
        assert(porter[0]>=0);
        if(first_section && porter[0]>last_time)
        {            
            xyzw= porter.block(4, 0, 4,1);
            mRawPoses.push_back(make_pair(porter[0],
                                SE3d(Quaterniond(xyzw.data()), porter.block(1,0,3,1))));
        }
        else
        {
            if(porter[0]<last_time){
                assert(first_section=true);
                first_section=false;
                mRawPoses[0].first= porter[0];
                cout<<"second session starts from:"<<porter.transpose()<<endl;
            }
            xyzw= porter.block(4, 0, 4,1);
            mKeyPoses.push_back(make_pair(porter[0],
                                SE3d(Quaterniond(xyzw.data()), porter.block(1,0,3,1))));
        }
        last_time=porter[0];
    }
    stream.close();
    cout<<"read in ordinary pose lines in the input file:"<< mRawPoses.size()-1
       <<" keyposes lines: "<< mKeyPoses.size()<<endl;
}
// a custom function for reading a particular data file
//in each data file, each row is index, P_{c_i}^w, 3x4 matrix, in row major order, vel s in w, ba, bg
void PostProcessor::LoadData2(const std::string sInputFile)
{
    Reset();
    ifstream stream(sInputFile.c_str());

    double last_time=-1;
    Eigen::Matrix<double,4,1> xyzw;

    const size_t nFields=22;
    Matrix<double, nFields, 1> proxy=Matrix<double, nFields, 1>::Zero();
    string tempStr;
    Matrix<double, 3,3 > Rc2w;
    Matrix<double,3,1> tcinw;
    getline(stream, tempStr); //remove first line
    while(!stream.eof()){
        for(size_t jack=0; jack<nFields; ++jack)
            stream>>proxy[jack];
        getline(stream, tempStr);
        if(stream.fail())
            break;

        assert( proxy[0]>last_time);

        Rc2w.row(0)= proxy.block(1, 0, 3,1).transpose();
        Rc2w.row(1)= proxy.block(5, 0, 3,1).transpose();
        Rc2w.row(2)= proxy.block(9, 0, 3,1).transpose();
        tcinw(0) = proxy[4];
        tcinw(1) = proxy[8];
        tcinw(2) = proxy[12];

        mRawPoses.push_back(make_pair(proxy[0],
                                SE3d(Rc2w, tcinw)));
        last_time=proxy[0];
    }
    stream.close();
    cout<<"read in ordinary pose lines in the input file:"<< mRawPoses.size()<<endl;
}
// a custom function for reading a particular data file
//in each data file, each row is index, P_{c_i}^w, 3x4 matrix, in row major order
// note the first line of the first section is for the second frame, the pose of the first frame is identity
void PostProcessor::LoadData3(const std::string sInputFile)
{
    Reset();
    ifstream stream(sInputFile.c_str());

    double last_time=-1;

    const size_t nFields=13;
    Matrix<double, nFields, 1> proxy=Matrix<double, nFields, 1>::Zero();
    string tempStr;
    Matrix<double, 3,3 > Rc2w;
    Matrix<double,3,1> tcinw;
//    getline(stream, tempStr); //remove first line
    mRawPoses.push_back(std::make_pair(0, SE3d()));//for the first frame
    while(!stream.eof()){
        for(size_t jack=0; jack<nFields; ++jack)
            stream>>proxy[jack];
        getline(stream, tempStr);
        if(stream.fail())
            break;

        assert( proxy[0]>last_time);

        Rc2w.row(0)= proxy.block(1, 0, 3,1).transpose();
        Rc2w.row(1)= proxy.block(5, 0, 3,1).transpose();
        Rc2w.row(2)= proxy.block(9, 0, 3,1).transpose();
        tcinw(0) = proxy[4];
        tcinw(1) = proxy[8];
        tcinw(2) = proxy[12];

        mRawPoses.push_back(make_pair(proxy[0],
                                SE3d(Rc2w, tcinw)));
        last_time=proxy[0];
    }
    stream.close();
    cout<<"read in ordinary pose lines in the input file:"<< mRawPoses.size()<<endl;
}
void PostProcessor:: ProcessPoses()
{
    mReprojedPoses=mRawPoses;
    SE3d Tc02neww= mKeyPoses.front().second;//new world frame is used for keyframes, old world frames are used in ordinary frames
    SE3d Tneww2c0= Tc02neww.inverse();
    SE3d Toldw2c0;
    SE3d Tprev2curr;
    SE3d lastTprev2curr;
    assert(mKeyPoses.front().first == mRawPoses.front().first);
    auto it_key= mKeyPoses.begin(), last_out= mReprojedPoses.end();
    auto  it_out= mReprojedPoses.begin();
    size_t counter=0;
    bool bUseReloc=false;
    for(auto it= mRawPoses.begin(), last_it= mRawPoses.end(), ite= mRawPoses.end();
        it!=ite; ++it, ++it_out, ++counter)
    {
        if(counter)
            Tprev2curr= it->second.inverse()*last_it->second;
        if(counter>1)
        {
            if(Tprev2curr.translation().norm()>1 && Tprev2curr.translation().norm()>2*lastTprev2curr.translation().norm())
            {
           //     cout<< "Tprev2curr.trans:"<< Tprev2curr.translation().transpose()<<endl;
            //    cout<< "lastTprev2curr.trans:"<< lastTprev2curr.translation().transpose()<<endl;
                cout<< "Relocalisation last frame and current frame time:"
                    << last_it->first <<" "<< it->first<<endl;
                bUseReloc=true;
                Tprev2curr= lastTprev2curr;
            }
        }
        if(it->first == it_key->first)// use keyframe poses to update poses
        {
            bUseReloc=false;
            it_out->second= Tneww2c0*it_key->second;
            Toldw2c0= it_out->second* it->second.inverse();
            ++it_key;
            while(it->first == it_key->first)//TODO in orbslam_dwo the second keyframe may be inserted twice, this is a remedy
                ++it_key;
        }
        else
        {
            if(bUseReloc)
                it_out->second = last_out->second* Tprev2curr.inverse();
            else
                it_out->second= Toldw2c0* it->second;//use relative poses to update poses
        }
        last_it= it;
        last_out= it_out;
        lastTprev2curr= Tprev2curr;
    }
    assert(it_key== mKeyPoses.end());
    assert(it_out== mReprojedPoses.end());
}
void PostProcessor::SaveData(const std::string sOutputFile)
{
    ofstream stream(sOutputFile);
    for(auto it= mReprojedPoses.begin(), ite= mReprojedPoses.end(); it!=ite; ++it)
    {
        Eigen::Matrix<double,3,4> proxy=it->second.matrix3x4();
        stream<<proxy.row(0)<<" "<< proxy.row(1)<<" "<< proxy.row(2) <<endl;
    }
    stream.close();
}

//eul [R;P;Y] defined in "n": rotate "n" to obtain "b"
//result: Cb2n (from b to n) s.t., Cb2n=R3(-Y)R2(-P)R1(-R)
template<class Scalar>
static Matrix<Scalar, 3,3 > roteu2ro( Matrix<Scalar, 3, 1> eul)
{
    Scalar cr = cos(eul[0]); Scalar sr = sin(eul[0]);	//roll
    Scalar cp = cos(eul[1]); Scalar sp = sin(eul[1]);	//pitch
    Scalar ch = cos(eul[2]); Scalar sh = sin(eul[2]);	//heading
    Matrix<Scalar, 3, 3> dcm;
    dcm.setZero();
    dcm(0,0) = cp * ch;
    dcm(0,1) = (sp * sr * ch) - (cr * sh);
    dcm(0,2) = (cr * sp * ch) + (sh * sr);

    dcm(1,0) = cp * sh;
    dcm(1,1) = (sr * sp * sh) + (cr * ch);
    dcm(1,2) = (cr * sp * sh) - (sr * ch);

    dcm(2,0) = -sp;
    dcm(2,1) = sr * cp;
    dcm(2,2) = cr * cp;
    return dcm;
}
// e.g., dir="/media/jianzhuhuai0108/Mag/NewTsukubaStereoDataset";
void InspectTsukubaEulerAngles(const string dir)
// the Tsukuba dataset gives unclear euler angles, this function displays trajectory
// of some point over frames, verifies the following:
//(1) Tsukuba camera coordinate system is defined as right, up and back for x,y,z
// when the we are viewing from behind of the camera
//(2) In camera_track.txt, T_{c_i}^{c_0} is given in each line, such that t_{c_i}^{c_0}=[X,Y,Z]',
// and R_{c_i}^{c_0}=R3(-C)R2(-B)R1(-A), and T_{c_i}^{c_0}=[R_{c_i}^{c_0}, t_{c_i}^{c_0}]
//(3) Its disparity map for occluded points are invalid
{
    using namespace cv;
    typedef SE3Group<double> SE3Type;
    //Tsukuba 3G stereo dataset
    string poseFile=dir+ "/groundtruth/camera_track.txt";
    //stores the transform Tc2w, stereo camera central frame to world, X,Y,Z and A,B,C of Euler Angles
    string leftImagePath=dir+"/illumination/daylight/left/tsukuba_daylight_L_";
    char leftImageName[300]={'\0'};
    string rightImagePath=dir+"/illumination/daylight/right/tsukuba_daylight_R_";
    char rightImageName[300]={'\0'};

    string leftImageDepth=dir+"/groundtruth/depth_maps/left/tsukuba_depth_L_00001.xml";
    string rightImageDepth=dir+"/groundtruth/depth_maps/right/tsukuba_depth_R_00001.xml";
    string leftImageDisp=dir+"/groundtruth/disparity_maps/left/tsukuba_disparity_L_00001.png";
    string rightImageDisp=dir+"/groundtruth/disparity_maps/right/tsukuba_disparity_R_00001.png";
    Vector3d point, imageObs;
    Matrix3d K;
    K<<615, 0, 320, 0, 615, 240, 0, 0, 1;
    imageObs<<600, 100, 1;
    point=K.inverse()*imageObs;
    cv::Mat leftDepth, rightDepth;
    cv::FileStorage fs;
    fs.open(leftImageDepth, cv::FileStorage::READ);
    if (!fs.isOpened())
    {
        cerr << "Failed to open " << leftImageDepth << endl;
        return;
    }
    fs["depth"] >> leftDepth;
    fs.release();
    fs.open(rightImageDepth, cv::FileStorage::READ);
    if (!fs.isOpened())
    {
        cerr << "Failed to open " << rightImageDepth << endl;
        return;
    }
    fs["depth"] >> rightDepth;
    fs.release();

    cout<<"left depth:"<<(leftDepth.at<float>(imageObs[1], imageObs[0])/100)<<endl;
    point*=(leftDepth.at<float>(imageObs[1], imageObs[0])/100);
    point[0]-=0.05;
    point[1]=-point[1];
    point[2]=-point[2];

    Mat leftDisp=cv::imread(leftImageDisp);
    Mat rightDisp=cv::imread(rightImageDisp);
    int dispL=leftDisp.at<uchar>(imageObs[1], imageObs[0]);
    cout<<"dispL:"<<dispL<<endl;
    cout<<"right depth:"<<rightDepth.at<float>(imageObs[1], imageObs[0]-dispL)/100<<endl;
    int dispR=rightDisp.at<uchar>(imageObs[1], imageObs[0]-dispL);
    cout<<"dispR:"<<dispR<<endl;
    sprintf(leftImageName, "%s%05d.png", leftImagePath.c_str(), 1);
    cv::Mat image=cv::imread(leftImageName);
//occlusion points may not correspond well according to their disparity
    cv::circle(image, Point(imageObs[0], imageObs[1]), 3, cv::Scalar(0,0,255), 2, 8, 0);
    cv::imshow("Point left", image);
    sprintf(rightImageName, "%s%05d.png", rightImagePath.c_str(), 1);
    image=cv::imread(rightImageName);

    cv::circle(image, cv::Point(imageObs[0]-dispL, imageObs[1]),
            3, cv::Scalar(0,0,255), 2, 8, 0);
    cv::imshow("Point right", image);
    cv::waitKey(0);
    cv::destroyAllWindows();
    std::vector<cv::Point2f> imagePointTraj;

    vector<SE3Type> q02n; //T_{c_i}^{c_0}, i=0,..., n, N=n+1

    Matrix<double,4,4> transMat;
    transMat<<1,0,0,0.05,
              0,-1,0,0,
              0,0,-1,0,
              0,0,0,1;
    SE3Type Tc2l(transMat); //transformation from center frame to left frame
    cerr<< "read in data poses"<<endl;
    ifstream dataptr(poseFile.c_str());
    assert(!dataptr.fail());
    string tempStr;

    Matrix<double, 3,1 > eulc2w;
    int lineNum=0;
    while(getline(dataptr,tempStr))
    {
        std::stringstream   linestream(tempStr);
        std::string        value;

        int valueNum=0;
        while(getline(linestream,value,','))
        {
            if(valueNum<3)
                transMat.data()[valueNum+12]=atof(value.c_str())/100;//convert from cm to m
            else
                eulc2w[valueNum-3]=atof(value.c_str())*M_PI/180;
            ++valueNum;
        }
        transMat.topLeftCorner(3,3)=roteu2ro(eulc2w);

        ++lineNum;
        q02n.push_back(SE3Type(transMat)); // this is Ts2s0
    }
    dataptr.close();
    cout<<"Num of lines:"<<lineNum<<endl;
    imagePointTraj.resize(lineNum);
    Vector3d tempPt;
    for( int j=0; j<lineNum; ++j)
    {
        tempPt=Tc2l*q02n[j].inverse()*point;
        if(tempPt[2]<0 || abs(tempPt[2])<1e-6)
        {
            imagePointTraj[j].x=-10;
            imagePointTraj[j].y=-10;
        }else{
        tempPt/=tempPt[2];
        tempPt=K*tempPt;
        imagePointTraj[j].x=tempPt[0];
        imagePointTraj[j].y=tempPt[1];
        }
    }
    for( int j=1; j<lineNum+1; ++j)
    {
        sprintf(leftImageName, "%s%05d.png", leftImagePath.c_str(), j);
        image=cv::imread(leftImageName);

        cv::circle(image, imagePointTraj[j-1], 3, cv::Scalar(0,0,255), 2, 8, 0);
        cv::imshow("Point Trajectory", image);
        cv::waitKey(33);
    }
}
// convert the ground truth file of Tsukuba dataset into KITTI format
void ConvertTsukubaGT2KITTIFormat(const string poseFile, const string sOutputFile)
{
    vector<SE3d> q02n; //T_{c_i}^{c_0}, i=0,..., n, N=n+1

    Matrix<double,4,4> transMat;
    transMat<<1,0,0,0.05,
              0,-1,0,0,
              0,0,-1,0,
              0,0,0,1;
    SE3d Tc2l(transMat); //transformation from center frame to left frame
    cout<< "read in data poses"<<endl;
    ifstream dataptr(poseFile.c_str());
    assert(!dataptr.fail());
    string tempStr;

    Matrix<double, 3,1 > eulc2w;
    int lineNum=0;
    while(getline(dataptr,tempStr))
    {
        std::stringstream   linestream(tempStr);
        std::string        value;

        int valueNum=0;
        while(getline(linestream,value,','))
        {
            if(valueNum<3)
                transMat.data()[valueNum+12]=atof(value.c_str())/100;//convert from cm to m
            else
                eulc2w[valueNum-3]=atof(value.c_str())*M_PI/180;
            ++valueNum;
        }
        transMat.topLeftCorner(3,3)=roteu2ro(eulc2w);
        ++lineNum;
        q02n.push_back(SE3d(transMat));
    }
    dataptr.close();
    cout<<"Num of lines:"<<lineNum<<endl;
    ofstream stream(sOutputFile);
    for(auto it= q02n.begin(), ite= q02n.end(); it!=ite; ++it)
    {
        SE3d q0l2nl= Tc2l*(*it)*Tc2l.inverse();
        Eigen::Matrix<double,3,4> proxy=q0l2nl.matrix3x4();
        stream<<proxy.row(0)<<" "<< proxy.row(1)<<" "<< proxy.row(2) <<endl;
    }
    stream.close();
    cout<< "Saved poses in KITTI ground truth format."<<endl;
}
