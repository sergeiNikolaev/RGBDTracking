/******************************************************************************
*       SOFA, Simulation Open-Framework Architecture, version 1.0 RC 1        *
*                (c) 2006-2011 MGH, INRIA, USTL, UJF, CNRS                    *
*                                                                             *
* This library is free software; you can redistribute it and/or modify it     *
* under the terms of the GNU Lesser General Public License as published by    *
* the Free Software Foundation; either version 2.1 of the License, or (at     *
* your option) any later version.                                             *
*                                                                             *
* This library is distributed in the hope that it will be useful, but WITHOUT *
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or       *
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License *
* for more details.                                                           *
*                                                                             *
* You should have received a copy of the GNU Lesser General Public License    *
* along with this library; if not, write to the Free Software Foundation,     *
* Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA.          *
*******************************************************************************
*                               SOFA :: Modules                               *
*                                                                             *
* Authors: The SOFA Team and external contributors (see Authors.txt)          *
*                                                                             *
* Contact information: contact@sofa-framework.org                             *
******************************************************************************/

#define SOFA_RGBDTRACKING_DATAIO_INL
#include <limits>
#include <iterator>
#include <sofa/core/behavior/ForceField.inl>

#include <pcl/common/common_headers.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>
#include <pcl/common/transforms.h>

#include "DataIO.h"

using std::cerr;
using std::endl;

namespace sofa
{

namespace core
{

namespace objectmodel
{
	
using namespace sofa::defaulttype;
using namespace helper;


template <class DataTypes>
DataIO<DataTypes>::DataIO()
    : Inherit()
    , useRealData(initData(&useRealData,true,"useRealData","Use real data"))
    , useSensor(initData(&useSensor,true,"useSensor","Use real data"))
    , inputPath(initData(&inputPath,"inputPath","Path for data readings",false))
    , outputPath(initData(&outputPath,"outputPath","Path for data writings",false))
    , dataPath(initData(&dataPath,"dataPath","Path for data writings",false))
    , nimages(initData(&nimages,"nimages","Number of images",false))
    , startimage(initData(&startimage,1,"startimage","Number of images"))
    , niterations(initData(&niterations,1,"niterations","Number of images"))
{
    this->f_listening.setValue(true);
}

template <class DataTypes>
DataIO<DataTypes>::~DataIO()
{
}

void writePCDToFile0(string path, std::vector<Vec3d>& pcd, std::vector<bool>& visible)
{

    //create the file stream
    ofstream file(path.c_str(), ios::out | ios::binary );
    double dvalue0,dvalue1,dvalue2;

    for (int k = 0; k < pcd.size(); k++)
    {
        dvalue0 = pcd[k][0];
        dvalue1 = pcd[k][1];
        dvalue2 = pcd[k][2];

        file << dvalue0;
        file << "\t";
        file << dvalue1;
        file << "\t";
        file << dvalue2;
        file << "\t";
        if (visible[k])
            file << 1;
        else file << 0;
        file << "\n";
    }

    file.close();

}


int readFileToPCD0(string path, std::vector<Vec3d>& pcd0, std::vector<bool>& visible0)
{

    //create the file stream
    ifstream file(path.c_str());
    double dvalue0,dvalue1,dvalue2;

    Vec3d point;
    bool vis;
    int visi;
    int nvisi = 0;

    while (file.good())
    {

        file >> dvalue0;
        file >> dvalue1;
        file >> dvalue2;
        file >> visi;
        if ((int)visi == 0) {vis = false;}
        else {vis = true; nvisi++;}

        //std::cout << " file " << dvalue0 << " " << dvalue1 << " " << dvalue2 << std::endl;

        point[0] = dvalue0;
        point[1] = dvalue1;
        point[2] = dvalue2;

        pcd0.push_back(point);
        visible0.push_back(vis);
    }

    file.close();

    return nvisi;

}

int writeMatToFile0(const cv::Mat &I, string path) {

    //load the matrix size
    int matWidth = I.size().width, matHeight = I.size().height;

    //read type from Mat
    int type = I.type();

    //declare values to be written
    float fvalue;
    double dvalue;
    Vec3f vfvalue;
    Vec3d vdvalue;

    //create the file stream
    ofstream file(path.c_str(), ios::out | ios::binary );
    if (!file)
        return -1;

    //write type and size of the matrix first
    file.write((const char*) &type, sizeof(type));
    file.write((const char*) &matWidth, sizeof(matWidth));
    file.write((const char*) &matHeight, sizeof(matHeight));

    //write data depending on the image's type
    switch (type)
    {
    default:
        cout << "Error: wrong Mat type: must be CV_32F, CV_64F, CV_32FC3 or CV_64FC3" << endl;
        break;
        // FLOAT ONE CHANNEL
    case CV_32F:
        //cout << "Writing CV_32F image" << endl;
        for (int i=0; i < matWidth*matHeight; ++i) {
            fvalue = I.at<float>(i);
            file.write((const char*) &fvalue, sizeof(fvalue));
        }
        break;
        // DOUBLE ONE CHANNEL
    case CV_64F:
        cout << "Writing CV_64F image" << endl;
        for (int i=0; i < matWidth*matHeight; ++i) {
            dvalue = I.at<double>(i);
            file.write((const char*) &dvalue, sizeof(dvalue));
        }
        break;

        // FLOAT THREE CHANNELS
    case CV_32FC3:
        cout << "Writing CV_32FC3 image" << endl;
        for (int i=0; i < matWidth*matHeight; ++i) {
            vfvalue = I.at<Vec3f>(i);
            file.write((const char*) &vfvalue, sizeof(vfvalue));
        }
        break;

        // DOUBLE THREE CHANNELS
    case CV_64FC3:
        cout << "Writing CV_64FC3 image" << endl;
        for (int i=0; i < matWidth*matHeight; ++i) {
            vdvalue = I.at<Vec3d>(i);
            file.write((const char*) &vdvalue, sizeof(vdvalue));
        }
        break;

    }

    //close file
    file.close();

    return 0;
}

int readFileToMat0(cv::Mat &I, string path) {

    //declare image parameters
    int matWidth, matHeight, type;

    matWidth = 0;
    matHeight = 0;

    //declare values to be written
    float fvalue;
    double dvalue;
    Vec3f vfvalue;
    Vec3d vdvalue;

    //create the file stream
    ifstream file(path.c_str(), ios::in | ios::binary );
    if (!file)
        return -1;

    //read type and size of the matrix first
    file.read((char*) &type, sizeof(type));
    file.read((char*) &matWidth, sizeof(matWidth));
    file.read((char*) &matHeight, sizeof(matHeight));

    std::cout << " width " << matWidth << " " << path <<  std::endl;

    //change Mat type
    I = cv::Mat::zeros(matHeight, matWidth, type);

    std::cout << " width " << matWidth << " " << matHeight <<  std::endl;

    //write data depending on the image's type
    switch (type)
    {
    default:
        cout << "Error: wrong Mat type: must be CV_32F, CV_64F, CV_32FC3 or CV_64FC3" << endl;
        break;
        // FLOAT ONE CHANNEL
    case CV_32F:
        cout << "Reading CV_32F image" << endl;
        for (int i=0; i < matWidth*matHeight; ++i) {
            file.read((char*) &fvalue, sizeof(fvalue));
            I.at<float>(i) = fvalue;
        }
        break;
        // DOUBLE ONE CHANNEL
    case CV_64F:
        cout << "Reading CV_64F image" << endl;
        for (int i=0; i < matWidth*matHeight; ++i) {
            file.read((char*) &dvalue, sizeof(dvalue));
            I.at<double>(i) = dvalue;
        }
        break;

        // FLOAT THREE CHANNELS
    case CV_32FC3:
        cout << "Reading CV_32FC3 image" << endl;
        for (int i=0; i < matWidth*matHeight; ++i) {
            file.read((char*) &vfvalue, sizeof(vfvalue));
            I.at<Vec3f>(i) = vfvalue;
        }
        break;

        // DOUBLE THREE CHANNELS
    case CV_64FC3:
        cout << "Reading CV_64FC3 image" << endl;
        for (int i=0; i < matWidth*matHeight; ++i) {
            file.read((char*) &vdvalue, sizeof(vdvalue));
            I.at<Vec3d>(i) = vdvalue;
        }
        break;

    }

    //close file
    file.close();

    return 0;
}

template<class DataTypes>
void DataIO<DataTypes>::readImages()
{
    int t = (int)this->getContext()->getTime();
    std::string opath = inputPath.getValue() + "/img1%06d.png";
    std::string opath1 = inputPath.getValue() + "/imgseg1%06d.png";
    std::string opath2 = inputPath.getValue() + "/depth1%06d.png";
    std::string opath3 = inputPath.getValue() + "/rtt%06d.png";
    std::string opath4 = inputPath.getValue() + "/depthfile%06d.txt";

    cv::Mat depth0,depth1;

    int niter = niterations.getValue();

    if (t%(npasses) == 0 )
    {
        if (t%niter==0)
        {
        char buf1[FILENAME_MAX];
        sprintf(buf1, opath.c_str(), iter_im);
        std::string filename1(buf1);

        char buf2[FILENAME_MAX];
        sprintf(buf2, opath1.c_str(), iter_im);
        std::string filename2(buf2);

        char buf3[FILENAME_MAX];
        sprintf(buf3, opath2.c_str(), iter_im);
        std::string filename3(buf3);

        char buf4[FILENAME_MAX];
        sprintf(buf4, opath4.c_str(), iter_im);
        std::string filename4(buf4);

        iter_im++;
        newImages.setValue(true);

        //if (t<1)
        color = cv::imread(filename1);
        wdth = color.cols;
        hght = color.rows;
        color_1 = color.clone();

        readFileToMat0(depth,filename4);
        cv::Mat color00;
        depth00 = depth.clone();
        resize(depth00, depth, Size(wdth, hght));
        color00 = color.clone();
        resize(color00, color, Size(wdth, hght));
        }
        else newImages.setValue(false);
    }
}


template <class DataTypes>
void DataIO<DataTypes>::init()
{
   iter_im = startimage.getValue();
   listimg.resize(0);
   listimgseg.resize(0);
   listimgklt.resize(0);
   listdepth.resize(0);
   listrtt.resize(0);

   listrttstress.resize(0);
   listrttstressplast.resize(0);
   listpcd.resize(0);
   listvisible.resize(0);
   pcl = false;
   disp = false;
   npasses = 1;
}

template <class DataTypes>
void DataIO<DataTypes>::handleEvent(sofa::core::objectmodel::Event *event)
{


    double time1 = (double)getTickCount();
    double timef0 = getTickCount();
        if (dynamic_cast<simulation::AnimateBeginEvent*>(event))
        {
            if (useRealData.getValue())
            {
                if(!useSensor.getValue())
                    readImages();
            }

            time1 = ((double)getTickCount() - time1)/getTickFrequency();
                            cout << "Time read images " << time1 << endl;

            int t = (int)this->getContext()->getTime();

             if (t == nimages.getValue()*niterations.getValue())
             {
                 if(useRealData.getValue())
                     writeImages();
             }

            double timeDataIO = ((double)getTickCount() - timef0)/getTickFrequency();

            std::cout << "TIME DATAIO " << timeDataIO << std::endl;
        }
}

template <class DataTypes>
void DataIO<DataTypes>::writeImages()
{	
    std::string opath = outputPath.getValue() + "/img1%06d.png";
    std::string opath1 = outputPath.getValue() + "/imgseg1%06d.png";
    std::string opath2 = outputPath.getValue() + "/depth1%06d.png";
    std::string opath3 = outputPath.getValue() + "/rtt%06d.png";
    std::string opath4 = outputPath.getValue() + "/depth2%06d.png";
    std::string opath5 = outputPath.getValue() + "/depthfile%06d.txt";
    std::string opath6 = outputPath.getValue() + "/rttstress%06d.png";
    std::string opath7 = outputPath.getValue() + "/rttstressplast%06d.png";
    std::string opath8 = outputPath.getValue() + "/imgklt%06d.png";

    cv::Mat img,img1,imgklt;
    cv::Mat imgseg,imgseg1;
    cv::Mat deptht,deptht1;
    cv::Mat rtt,rtt1, rttstress, rttstressplast;

    for (int frame_count = 0 ;frame_count < listimgseg.size()-5; frame_count++)
    {

        std::cout << " ok write images " << frame_count << std::endl;
        img = *listimg[frame_count];
        imgseg = *listimgseg[frame_count];
        deptht = *listdepth[frame_count];
        cvtColor(imgseg,imgseg1 ,CV_RGBA2RGB);
        deptht.convertTo (deptht1, CV_8UC1, 100);

        char buf1[FILENAME_MAX];
        sprintf(buf1, opath.c_str(), frame_count);
        std::string filename1(buf1);

        char buf2[FILENAME_MAX];
        sprintf(buf2, opath1.c_str(), frame_count);
        std::string filename2(buf2);

        char buf3[FILENAME_MAX];
        sprintf(buf3, opath2.c_str(), frame_count);
        std::string filename3(buf3);

        char buf4[FILENAME_MAX];
        sprintf(buf4, opath4.c_str(), frame_count);
        std::string filename4(buf4);

        char buf5[FILENAME_MAX];
        sprintf(buf5, opath5.c_str(), frame_count);
        std::string filename5(buf5);

        cv::imwrite(filename1,img);
        cv::imwrite(filename2,imgseg);
        cv::imwrite(filename3,deptht1);
        cv::imwrite(filename4,deptht);

        writeMatToFile0(deptht,filename5);

        if (useKLTPoints.getValue()){
            char buf8[FILENAME_MAX];
            sprintf(buf8, opath8.c_str(), frame_count);
            std::string filename8(buf8);
            imgklt = *listimgklt[frame_count];
            cv::imwrite(filename8,imgklt);
        }

        delete listimg[frame_count];
        delete listimgseg[frame_count];
        delete listdepth[frame_count];
    }


    for (int frame_count = 1 ;frame_count < listrtt.size(); frame_count++)
    {
        std::cout << " ok write rtt " << frame_count << std::endl;
        rtt = *listrtt[frame_count];
        cvtColor(rtt,rtt1 ,CV_RGB2BGR);
        char buf4[FILENAME_MAX];
        sprintf(buf4, opath3.c_str(), frame_count-1);
        std::string filename4(buf4);
        cv::imwrite(filename4,rtt1);

        if (npasses == 2)
        {
            rtt = *listrttstress[frame_count];
            cvtColor(rtt,rttstress ,CV_RGB2BGR);
            char buf6[FILENAME_MAX];
            sprintf(buf6, opath6.c_str(), frame_count - 1);
            std::string filename6(buf6);
            cv::imwrite(filename6,rttstress);

        }
        else if (npasses == 2){
            rtt = *listrttstressplast[frame_count];
            cvtColor(rtt,rttstressplast ,CV_RGB2BGR);
            char buf7[FILENAME_MAX];

            sprintf(buf7, opath7.c_str(), frame_count - 1);
            std::string filename7(buf7);
            cv::imwrite(filename7,rttstressplast);

        }
        delete listrtt[frame_count];
    }
}

}
}
} // namespace sofa

