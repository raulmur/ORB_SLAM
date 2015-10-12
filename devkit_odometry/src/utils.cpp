#include "utils.h"
#include <stdlib.h> //"system"
#include <stdio.h> //fprintf
#include <fstream> //ofstream
using namespace std;

// plot paths of ground truth and produced by different odometry methods
// example: vector<string> dir_all; customPlotPathPlot (dir_all);

void customPlotPathPlot (vector<string> dir_all) {
    dir_all.clear();
    dir_all.push_back("/media/jhuai/Mag/kitti/viso206_mm_dwo_4_8");// new dwo with motion model
    dir_all.push_back("/media/jhuai/Mag/kitti/viso210_dwo_3_7"); // old dwo
//    dir_all.push_back("/media/jhuai/Mag/kitti/viso210");// viso2
    dir_all.push_back("/media/jhuai/Mag/kitti/viso206_mm_subp_dwo_4_8");//new dwo with motion model and subpixel refinement
//    dir_all.push_back("/media/jhuai/Mag/kitti/qcv06_dwo_4_8");// qcv scale 06 and dwo 4 temporal window_8 spatial window
    dir_all.push_back("/home/jhuai/qcv/qcv-mods/build/kittisettings/result_scale06");// stereo sfm
    dir_all.push_back("/media/jhuai/Mag/kitti/data_odometry_poses/dataset/poses");//ground truth

    dir_all.push_back("/media/jhuai/Mag/kitti/viso206_mm_subp_procviso2_dwo_4_8");// new dwo with motion model, subpixel refinement without stereo matching
    string dir= "/home/jhuai/orbslam_dwo/Data/result/plot_path";//where to put results
    // gnuplot file name
    char command[1024];
    char file_name[256];
    for (int idx=0; idx<22; ++idx){

        sprintf(file_name,"%02d.gp",idx);

        string full_name = dir +"/"+ file_name;
        // create png + eps
        for (int32_t i=0; i<2; i++) {

            // open file
            FILE *fp = fopen(full_name.c_str(),"w");

            // save gnuplot instructions
            if (i==0) {
                fprintf(fp,"set term png size 900,900\n");
                fprintf(fp,"set output \"%02d.png\"\n",idx);
            } else {
                fprintf(fp,"set term postscript eps enhanced color\n");
                fprintf(fp,"set output \"%02d.eps\"\n",idx);
            }

            fprintf(fp,"set size ratio -1\n");
            //    fprintf(fp,"set xrange [%d:%d]\n",roi[0],roi[1]);
            //    fprintf(fp,"set yrange [%d:%d]\n",roi[2],roi[3]);
            fprintf(fp,"set xlabel \"x [m]\"\n");
            fprintf(fp,"set ylabel \"z [m]\"\n");
            if(idx >15)
              fprintf(fp,"plot \"%s/%02d.txt\" using 4:12 lc rgb \"#0000FF\" title 'viso2 0.6 mm DWO 4 8' w lines,",dir_all[0].c_str(), idx);
            else if(idx>10)
            {
                fprintf(fp,"plot \"%s/%02d.txt\" using 1:2 lc rgb \"#FF0000\" title 'Ground Truth' w lines,",dir_all[4].c_str(), idx);
                fprintf(fp,"\"%s/%02d.txt\" using 4:12 lc rgb \"#0000FF\" title 'viso2 0.6 mm DWO 4 8' w lines,",dir_all[0].c_str(), idx);
            }
            else{
            fprintf(fp,"plot \"%s/%02d.txt\" using 4:12 lc rgb \"#FF0000\" title 'Ground Truth' w lines,",dir_all[4].c_str(), idx);
            fprintf(fp,"\"%s/%02d.txt\" using 4:12 lc rgb \"#0000FF\" title 'viso2 0.6 mm DWO 4 8' w lines,",dir_all[0].c_str(), idx);
            }
            fprintf(fp,"\"%s/%02d.txt\" using 4:12 lc rgb \"#FF00FF\" title 'viso2 1.0 DWO 3 7' w lines,",dir_all[1].c_str(), idx);
//            fprintf(fp,"\"%s/%02d.txt\" using 4:12 lc rgb \"#000000\" title 'viso2 1.0' w lines,",dir_all[2].c_str(), idx);
//            fprintf(fp,"\"%s/%02d.txt\" using 4:12 lc rgb \"#000000\" title 'qcv 0.6 dwo 4 6' w lines,",dir_all[2].c_str(), idx);
            fprintf(fp,"\"%s/KITTISeq%02d.txt\" using 2:4 lc rgb \"#000000\" title 'viso2 0.6 mm subp dwo 4 6' w lines,",dir_all[2].c_str(), idx);
            fprintf(fp,"\"%s/%02d.txt\" using 4:12 lc rgb \"#008000\" title 'SSFM' w lines,",dir_all[3].c_str(), idx);
            fprintf(fp,"\"%s/%02d.txt\" using 4:12 lc rgb \"#0FAF00\" title 'viso2 0.6 mm subp viso2 DWO 4 8' w lines,",dir_all[5].c_str(), idx);
            fprintf(fp,"\"< head -1 %s/%02d.txt\" using 4:12 lc rgb \"#000000\" pt 4 ps 1 lw 2 title 'Sequence Start' w points\n",dir_all[3].c_str(),idx);

            // close file
            fclose(fp);

            // run gnuplot => create png + eps
            sprintf(command,"cd %s; gnuplot %s",dir.c_str(),file_name);
            system(command);
        }
    }
}
void customShellFile(std::string path, std::string outputShell, std::vector<int> seqNum)
{
     path= "/home/jhuai/qcv/qcv-mods/build/kittisettings";
     outputShell ="/home/jhuai/qcv/qcv-mods/build/kitti.sh";
     seqNum.clear();
     for (int jack=0; jack<22; ++jack)
        seqNum.push_back(jack);

     ofstream stream(outputShell);
     stream<<"cd ./bin"<<endl;
     char suffix[16]={'/0'};
     string paramFile= path + "/params_stereoSFM.xml";
     for (size_t jack=0; jack<seqNum.size(); ++jack){
         sprintf(suffix,"%02d",seqNum[jack]);
         string seqSetting= path + "/sequence_kitti" + suffix +".xml";
         string resTraj= path +"/result/" + suffix +".txt";
         string resTracks =path +"/result/"+suffix +"_tracks.txt";
         string resDelta =path +"/result/" +suffix +"_Tci2cip1.txt";
     stream<<"./stereoSFM "<<seqSetting<<' '<<paramFile<<' '<<
             resTraj<<' '<<resTracks<<' '<<resDelta<<' '<<
             "--autoplay --nowindows"<<endl;
     }
     stream.close();
}
