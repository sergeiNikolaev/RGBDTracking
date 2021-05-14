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

#define SOFA_RGBDTRACKING_RGBDDATAPROCESSING_FORKALMANFILTER_INL


#include "RGBDDataProcessingForKalmanFilter.h"

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



/**
 *        RGBDDataProcessingForKalmanFilter::RGBDDataProcessingForKalmanFilter
 * <p>
 *   description:
 *       constructor
 * @see none
 *
 *  arguments:
 * @param  none
 *
 * @return  none
 *
 */
template <class DataTypes>
RGBDDataProcessingForKalmanFilter<DataTypes>::RGBDDataProcessingForKalmanFilter( )
 : RGBDDataProcessing<DataTypes>()
 , d_controlMask( initData(&d_controlMask, "controlMask", "mask to detect control points") )
 , d_controlMaskIsNotExtracted( initData(&d_controlMaskIsNotExtracted, true, "controlMaskExtracted", "show if control mask is extracted") )
{

}



/**
 *        RGBDDataProcessingForKalmanFilter::init
 * <p>
 *   description:
 *     initialise component
 * @see none
 *
 *  arguments:
 * @param  none
 *
 * @return  none
 *
 */
template <class DataTypes>
void RGBDDataProcessingForKalmanFilter<DataTypes>::init()
{
    RGBDDataProcessing<DataTypes>::init();
}



/**
 *        MeshProcessingForKalmanFilter::initSegmentation
 * <p>
 *   description:
 *     start  image segmentation process
 * @see none
 *
 *  arguments:
 * @param  none
 *
 * @return  none
 *
 */
template <class DataTypes>
void RGBDDataProcessingForKalmanFilter<DataTypes>::initSegmentation()
{

    cv::Mat mask,maskimg,mask0,roimask,mask1; // segmentation result (4 possible values)
    cv::Mat bgModel,fgModel; // the models (internally used)

    cv::Mat downsampledbox,downsampled;

    int scaleSeg = this->scaleSegmentation.getValue();
    if (scaleSeg>1)
        cv::resize(this->color, downsampledbox, cv::Size(this->color.cols/scaleSeg, this->color.rows/scaleSeg));
    else
        downsampledbox = this->color.clone();

    //cv::imwrite("colorinit.png", color);

    cv::Mat temp;
    cv::Mat tempgs;
    cv::Mat imgs,imgklt;
    temp = downsampledbox.clone();
    cv::Mat temp1 = temp.clone();

    const char* name = "image";

    cv::namedWindow(name);
    RGBDDataProcessing<DataTypes>::box = cvRect(0,0,1,1);

    //cv::imshow("image",	color);
    //tempm.resize(image.step1());

    // Set up the callback
    cv::setMouseCallback(name, my_mouse_callback_forKalmanFilter, (void*) &temp);

    std::cout << " Time " << (int)this->getContext()->getTime() << std::endl;

    // Main loop
    while(1)
    {
      if (this->destroy)
      {
        cv::destroyWindow(name); break;
      }
      temp1 = temp.clone();

      if (this->drawing_box)
          if (control_selection() == true) {
              RGBDDataProcessingForKalmanFilter<DataTypes>::draw_control_box(temp1);
          } else {
              RGBDDataProcessing<DataTypes>::draw_box(temp1, this->box);
          }

      cv::moveWindow(name, 200, 100);
      cv::imshow(name, temp1);
      //tempm.resize(image.step1());
      int key=waitKey(10);
      if ((char)key == 'n' || (char)key == 'N')
      {
          control_selection() = true;
          control_box_index()++;
          box_vector().push_back(cvRect(0,0,0,0));
      }
      else if ((char)key == 27) break;

    }
   // delete temp1;
    temp1.release();
    cv::setMouseCallback(name, NULL, NULL);

    //cvDestroyWindow(name);
    cv::Rect rectangle(69,47,198,171);

    rectangle = this->box;
    this->seg.setRectangle(rectangle);

    if (scaleSeg>1)
    cv::resize(this->color, downsampled, cv::Size(this->color.cols/scaleSeg, this->color.rows/scaleSeg));
    else downsampled = this->color.clone();

    this->foregroundS = cv::Mat(downsampled.size(),CV_8UC3,cv::Scalar(255,255,255));

    this->seg.segmentationFromRect(downsampled, this->foregroundS);

    cv::resize(this->foregroundS, this->foreground, this->color.size());

    // draw rectangle on original image
    //cv::rectangle(image, rectangle, cv::Scalar(255,255,255),1);

    // display result
    if (this->displaySegmentation.getValue()){
        cv::imshow("image_segmented", this->foregroundS);
        cv::waitKey(1);
    }

    if (waitKey(20) == 27) //wait for 'esc' key press for 30ms. If 'esc' key is pressed, break loop
    {
        cout << "esc key is pressed by user" << endl;
    }

}



/**
 *        MeshProcessingForKalmanFilter::extractTargetPCD
 * <p>
 *   description:
 *     create 3d point cloud from RGB and depth images
 * @see none
 *
 *  arguments:
 * @param  none
 *
 * @return  none
 *
 */
template <class DataTypes>
void RGBDDataProcessingForKalmanFilter<DataTypes>::extractTargetPCD()
{
    this->targetP.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
    this->targetP = PCDFromRGBD(this->depth,this->foreground);
    VecCoord targetpos;

    if (this->targetP->size() > 10)
    {
        this->target.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
        this->target = this->targetP;
        targetpos.resize(this->target->size());

        Vector3 pos;
        Vector3 col;

        for (unsigned int i=0; i<this->target->size(); i++)
        {
            pos[0] = (double)this->target->points[i].x;
            pos[1] = (double)this->target->points[i].y;
            pos[2] = (double)this->target->points[i].z;
            targetpos[i]=pos;
            //std::cout << " target " << pos[0] << " " << pos[1] << " " << pos[2] << std::endl;
        }
        const VecCoord&  p = targetpos;
        this->targetPositions.setValue(p);
    }
}



/**
 *        MeshProcessingForKalmanFilter::PCDFromRGBD
 * <p>
 *   description:
 *     generate 3d point cloud
 * @see none
 *
 *  arguments:
 * @param  none
 *
 * @return  generated 3d point cloud
 *
 */
template <class DataTypes>
pcl::PointCloud<pcl::PointXYZRGB>::Ptr RGBDDataProcessingForKalmanFilter<DataTypes>::PCDFromRGBD(cv::Mat& depthImage, cv::Mat& rgbImage)
{
    helper::WriteAccessor<Data<helper::vector<unsigned int>>> vControlMask = d_controlMask;
    helper::vector<unsigned int> currentMask;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr outputPointcloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    //pcl::PointCloud<pcl::PointXYZRGB> pointcloud;
    outputPointcloud->points.resize(0);

    int sample;

    switch (this->sensorType.getValue())
    {
    // FLOAT ONE CHANNEL
    case 0:
	sample = 2;
	break;
	case 1:

        sample = this->samplePCD.getValue();
	break;
        }

	float rgbFocalInvertedX = 1/this->rgbIntrinsicMatrix(0,0);	// 1/fx
        float rgbFocalInvertedY = 1/this->rgbIntrinsicMatrix(1,1);	// 1/fy
	pcl::PointXYZRGB newPoint;

        for (int i=0;i<(int)depthImage.rows/sample;i++)
	{
                for (int j=0;j<(int)depthImage.cols/sample;j++)
		{

			float depthValue = (float)depthImage.at<float>(sample*i,sample*j);//*0.819;
			int avalue = (int)rgbImage.at<Vec4b>(sample*i,sample*j)[3];
			if (avalue > 0 && depthValue>0)                // if depthValue is not NaN
			{
				// Find 3D position respect to rgb frame:
                                newPoint.z = depthValue;
				newPoint.x = (sample*j - this->rgbIntrinsicMatrix(0,2)) * newPoint.z * rgbFocalInvertedX;
				newPoint.y = (sample*i - this->rgbIntrinsicMatrix(1,2)) * newPoint.z * rgbFocalInvertedY;
				newPoint.r = rgbImage.at<cv::Vec4b>(sample*i,sample*j)[2];
				newPoint.g = rgbImage.at<cv::Vec4b>(sample*i,sample*j)[1];
				newPoint.b = rgbImage.at<cv::Vec4b>(sample*i,sample*j)[0];
				outputPointcloud->points.push_back(newPoint);

				if (d_controlMaskIsNotExtracted.getValue() == true) {
				    for (size_t index = 0; index < box_vector().size(); index++) {
				        cv::Rect currentBox = box_vector()[index];
				        if ((sample*i > currentBox.y) && (sample*i < currentBox.y + currentBox.height) &&
				                (sample*j > currentBox.x) && (sample*j < currentBox.x + currentBox.width)) {
				            currentMask.push_back(1);
				        } else {
				            currentMask.push_back(0);
				        }
				    }
				}

			}


	         }

          }

    if (this->useGroundTruth.getValue())
    {
        int sample1 = 2;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr outputPointcloud1(new pcl::PointCloud<pcl::PointXYZRGB>);

        for (int i=0;i<(int)depthImage.rows/sample1;i++)
	{
                for (int j=0;j<(int)depthImage.cols/sample1;j++)
		{
                        float depthValue = (float)depthImage.at<float>(sample1*i,sample1*j);//*0.819;
			//depthValue =  1.0 / (depthValue*-3.0711016 + 3.3309495161);;
			int avalue = (int)rgbImage.at<Vec4b>(sample1*i,sample1*j)[3];
			if (avalue > 0 && depthValue>0)                // if depthValue is not NaN
			{
				// Find 3D position respect to rgb frame:
				newPoint.z = depthValue;
				newPoint.x = (sample1*j - this->rgbIntrinsicMatrix(0,2)) * newPoint.z * rgbFocalInvertedX;
				newPoint.y = (sample1*i - this->rgbIntrinsicMatrix(1,2)) * newPoint.z * rgbFocalInvertedY;
				newPoint.r = rgbImage.at<cv::Vec4b>(sample1*i,sample1*j)[2];
				newPoint.g = rgbImage.at<cv::Vec4b>(sample1*i,sample1*j)[1];
				newPoint.b = rgbImage.at<cv::Vec4b>(sample1*i,sample1*j)[0];
				outputPointcloud1->points.push_back(newPoint);


			}

		}
	}

        this->targetPointCloud =outputPointcloud1;

    }

    if (this->useCurvature.getValue())
    {
        int sample1 = this->samplePCD.getValue();//3;
        pcl::PointCloud<pcl::PointXYZ>::Ptr outputPointcloud1(new pcl::PointCloud<pcl::PointXYZ>);
        outputPointcloud1->points.resize(0);

        pcl::PointXYZ newPoint1;

         for (int i=0;i<(int)depthImage.rows/sample1;i++)
        {
                for (int j=0;j<(int)depthImage.cols/sample1;j++)
                {
                        float depthValue = (float)depthImage.at<float>(sample1*i,sample1*j);//*0.819;
                        int avalue = (int)rgbImage.at<Vec4b>(sample1*i,sample1*j)[3];
                        if (avalue > 0 && depthValue>0)                // if depthValue is not NaN
                        {
                                // Find 3D position respect to rgb frame:
                                newPoint1.z = depthValue;
                                newPoint1.x = (sample1*j - this->rgbIntrinsicMatrix(0,2)) * newPoint.z * rgbFocalInvertedX;
                                newPoint1.y = (sample1*i - this->rgbIntrinsicMatrix(1,2)) * newPoint.z * rgbFocalInvertedY;
                                outputPointcloud1->points.push_back(newPoint1);


                        }

                }
        }


        // Compute the normals
        pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimation;
        normalEstimation.setInputCloud (outputPointcloud1);
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
        normalEstimation.setSearchMethod (tree);
        pcl::PointCloud<pcl::Normal>::Ptr cloudWithNormals (new pcl::PointCloud<pcl::Normal>);
        normalEstimation.setRadiusSearch (0.02);
        normalEstimation.compute (*cloudWithNormals);


        /*vector<pcl::PointXYZ> normals_for_gpu(cloudWithNormals->points.size());
        std::transform(cloudWithNormals->points.begin(), cloudWithNormals->points.end(), normals_for_gpu.begin(), pcl::gpu::DataSource::Normal2PointXYZ());


        pcl::gpu::PrincipalCurvaturesEstimation::PointCloud cloud_gpu;
            cloud_gpu.upload(outputPointcloud1->points);

            pcl::gpu::PrincipalCurvaturesEstimation::Normals normals_gpu;
            normals_gpu.upload(normals_for_gpu);

            pcl::gpu::DeviceArray<pcl::PrincipalCurvatures> pc_features;

            pcl::gpu::PrincipalCurvaturesEstimation pc_gpu;
            pc_gpu.setInputCloud(cloud_gpu);
            pc_gpu.setInputNormals(normals_gpu);
            pc_gpu.setRadiusSearch(0.03, 50);
            pc_gpu.compute(pc_features);

            vector<pcl::PrincipalCurvatures> downloaded;
            pc_features.download(downloaded);

            pcl::PrincipalCurvaturesEstimation<pcl::PointXYZ, pcl::Normal, pcl::PrincipalCurvatures> fe;
            fe.setInputCloud (outputPointcloud1);
            fe.setInputNormals (cloudWithNormals);
            fe.setRadiusSearch(1);

            pcl::PointCloud<pcl::PrincipalCurvatures> pc;
            fe.compute (pc);

            for(size_t i = 0; i < downloaded.size(); ++i)
            {
                pcl::PrincipalCurvatures& gpu = downloaded[i];
                pcl::PrincipalCurvatures& cpu = pc.points[i];

            }*/

        // Setup the principal curvatures computation
        pcl::PrincipalCurvaturesEstimation<pcl::PointXYZ, pcl::Normal, pcl::PrincipalCurvatures> principalCurvaturesEstimation;

        // Provide the original point cloud (without normals)
        principalCurvaturesEstimation.setInputCloud (outputPointcloud1);

        // Provide the point cloud with normals
        principalCurvaturesEstimation.setInputNormals(cloudWithNormals);

        // Use the same KdTree from the normal estimation
        principalCurvaturesEstimation.setSearchMethod (tree);
        principalCurvaturesEstimation.setRadiusSearch(0.03);

        // Actually compute the principal curvatures
        pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr principalCurvatures (new pcl::PointCloud<pcl::PrincipalCurvatures> ());
        principalCurvaturesEstimation.compute (*principalCurvatures);

        std::cout << "output points.size (): " << principalCurvatures->points.size () << std::endl;

        // Display and retrieve the shape context descriptor vector for the 0th point.
        pcl::PrincipalCurvatures descriptor = principalCurvatures->points[0];

        std::vector<double> curvs;

        for (int k = 0; k < principalCurvatures->points.size (); k++)
        {
            pcl::PrincipalCurvatures descriptor0 = principalCurvatures->points[k];

            double curv = abs(descriptor0.pc1*descriptor0.pc1);
            curvs.push_back(curv);
        }

        this->curvatures.setValue(curvs);

    }

    if (d_controlMaskIsNotExtracted.getValue() == true) {
        d_controlMaskIsNotExtracted.setValue(false);
        vControlMask.resize(currentMask.size());
        for (size_t index = 0; index < currentMask.size(); index++) {
            vControlMask[index] = currentMask[index];
        }
    }

    return outputPointcloud;
}



/**
 *        MeshProcessingForKalmanFilter::draw_control_box
 * <p>
 *   description:
 *     draw method to show controls selection
 * @see none
 *
 *  arguments:
 * @param  _img - a pointer to image to draw the selection
 *
 * @return  none
 *
 */
template <class DataTypes>
void RGBDDataProcessingForKalmanFilter<DataTypes>::draw_control_box(cv::Mat _img)
{
    for (size_t index = 0; index < box_vector().size(); index++) {
        cv::rectangle(_img, cvPoint(box_vector()[control_box_index()].x, box_vector()[control_box_index()].y),
            cvPoint(box_vector()[control_box_index()].x+box_vector()[control_box_index()].width, box_vector()[control_box_index()].y+box_vector()[control_box_index()].height), cvScalar(0,255,0), 2);
    }

    //cv::Rect rect2=cv::Rect(RGBDDataProcessing<DataTypes>::box.x,RGBDDataProcessing<DataTypes>::box.y,RGBDDataProcessing<DataTypes>::box.width,RGBDDataProcessing<DataTypes>::box.height);
    //cvSetImageROI(image, rect2);   //here I wanted to set the drawn rect as ROI
}



/**
 *        MeshProcessingForKalmanFilter::my_mouse_callback_forKalmanFilter
 * <p>
 *   description:
 *     modified mouse callback in case of several selections
 * @see none
 *
 *  arguments:
 * @param   - mouse params
 *
 * @return  none
 *
 */
template <class DataTypes>
void RGBDDataProcessingForKalmanFilter<DataTypes>::my_mouse_callback_forKalmanFilter( int event, int x, int y, int flags, void* param )
{
  cv::Mat* frame = (cv::Mat*) param;

  switch( event )
  {
      case CV_EVENT_MOUSEMOVE:
      {
          if( RGBDDataProcessing<DataTypes>::drawing_box )
          {
              if (control_selection() == true) {
                  box_vector()[control_box_index()].width = x-box_vector()[control_box_index()].x;
                  box_vector()[control_box_index()].height = y-box_vector()[control_box_index()].y;
              } else {
                  RGBDDataProcessing<DataTypes>::box.width = x-RGBDDataProcessing<DataTypes>::box.x;
                  RGBDDataProcessing<DataTypes>::box.height = y-RGBDDataProcessing<DataTypes>::box.y;
              }
          }
      }
      break;

      case CV_EVENT_LBUTTONDOWN:
      {
          RGBDDataProcessing<DataTypes>::drawing_box = true;
          if (control_selection() == true) {
              box_vector()[control_box_index()] = cvRect( x, y, 0, 0 );
          } else {
              RGBDDataProcessing<DataTypes>::box = cvRect( x, y, 0, 0 );
          }

      }
      break;

      case CV_EVENT_LBUTTONUP:
      {
          RGBDDataProcessing<DataTypes>::drawing_box = false;
          if (control_selection() == true) {
              if( box_vector()[control_box_index()].width < 0 )
              {
                  box_vector()[control_box_index()].x += box_vector()[control_box_index()].width;
                  box_vector()[control_box_index()].width *= -1;
              }

              if( box_vector()[control_box_index()].height < 0 )
              {
                  box_vector()[control_box_index()].y += box_vector()[control_box_index()].height;
                  box_vector()[control_box_index()].height *= -1;
              }

              RGBDDataProcessingForKalmanFilter<DataTypes>::draw_control_box(*frame);
          } else {
              if( RGBDDataProcessing<DataTypes>::box.width < 0 )
              {
                  RGBDDataProcessing<DataTypes>::box.x += RGBDDataProcessing<DataTypes>::box.width;
                  RGBDDataProcessing<DataTypes>::box.width *= -1;
              }

              if( RGBDDataProcessing<DataTypes>::box.height < 0 )
              {
                  RGBDDataProcessing<DataTypes>::box.y += RGBDDataProcessing<DataTypes>::box.height;
                  RGBDDataProcessing<DataTypes>::box.height *= -1;
              }

              RGBDDataProcessing<DataTypes>::draw_box(*frame, RGBDDataProcessing<DataTypes>::box);
          }
      }
      break;

      case CV_EVENT_RBUTTONUP:
      {
          RGBDDataProcessing<DataTypes>::destroy = true;
      }
      break;

      default:
      break;
   }

}


/**
 *        MeshProcessingForKalmanFilter::handleEvent
 * <p>
 *   description:
 *     a methood that does something
 * @see none
 *
 *  arguments:
 * @param  event - a pointer to some event
 *
 * @return  none
 *
 */
template <class DataTypes>
void RGBDDataProcessingForKalmanFilter<DataTypes>::handleEvent(sofa::core::objectmodel::Event *event)
{
    if (dynamic_cast<simulation::AnimateBeginEvent*>(event))
    {
        double timeT = (double)getTickCount();
        double timeAcq0 = (double)getTickCount();

        sofa::simulation::Node::SPtr root = dynamic_cast<simulation::Node*>(this->getContext());
        typename sofa::core::objectmodel::ImageConverter<DataTypes,DepthTypes>::SPtr imconv;
        root->get(imconv);

        typename sofa::core::objectmodel::DataIO<DataTypes>::SPtr dataio;
        root->get(dataio);

        bool okimages = false;

        if (this->useRealData.getValue())
        {
            if (this->useSensor.getValue())
            {
                //color_1 = color.clone();
                //depth_1 = depth.clone();
                if(!((imconv->depth).empty()) && !((imconv->color).empty()))
                {
                    if (this->scaleImages.getValue() > 1)
                    {
                        cv::resize(imconv->depth, this->depth, cv::Size(imconv->depth.cols/this->scaleImages.getValue(), imconv->depth.rows/this->scaleImages.getValue()), 0, 0);
                        cv::resize(imconv->color, this->color, cv::Size(imconv->color.cols/this->scaleImages.getValue(), imconv->color.rows/this->scaleImages.getValue()), 0, 0);
                    }
                    else
                    {
                        this->color = imconv->color;
                        this->depth = imconv->depth;
                    }
                    okimages = true;
                }
                //cv::imwrite("depth22.png", depth);
            }
            else
            {
                this->color = dataio->color;
                this->depth = dataio->depth;
                this->color_1 = dataio->color_1;
                okimages = true;
            }

            double timeAcq1 = (double)getTickCount();
            cout <<"TIME GET IMAGES " << (timeAcq1 - timeAcq0)/getTickFrequency() << endl;

            this->imagewidth.setValue(this->color.cols);
            this->imageheight.setValue(this->color.rows);


            if (this->displayImages.getValue() && this->displayDownScale.getValue() > 0 && !this->depth.empty() && !this->color.empty())
            {
                int scale = this->displayDownScale.getValue();
                cv::Mat colorS, depthS;
                cv::resize(this->depth, depthS, cv::Size(this->imagewidth.getValue()/scale, this->imageheight.getValue()/scale), 0, 0);
                cv::resize(this->color, colorS, cv::Size(this->imagewidth.getValue()/scale, this->imageheight.getValue()/scale), 0, 0);

                /*cv::Mat depthmat1;
                depthS.convertTo(depthmat1, CV_8UC1, 255);
                cv::imwrite("depthS0.png", depthmat1);*/
                cv::imshow("image_sensor",colorS);
                cv::waitKey(1);
                cv::imshow("depth_sensor",depthS);
                cv::waitKey(1);
            }

            if (this->saveImages.getValue())
            {
                cv::Mat* imgl = new cv::Mat;
                *imgl = this->color.clone();
                cv::Mat* depthl = new cv::Mat;
                *depthl = this->depth.clone();

                dataio->listimg.push_back(imgl);
                dataio->listdepth.push_back(depthl);
            }
        }
        else
        {
            dataio->readData();
            okimages=true;
        }


        if (okimages)
        {
            if (this->initsegmentation)
            {

                if (this->useRealData.getValue())
                {
                    initSegmentation();
                    extractTargetPCD();
                }
                setCameraPose();
                this->initsegmentation = false;
            }
            else
            {
                if(this->useRealData.getValue())
                {
                    segment();
                    this->timePCD = (double)getTickCount();

                    if(!this->useContour.getValue())
                        extractTargetPCD();
                    else
                        extractTargetPCDContour();

                    this->timePCD = ((double)getTickCount() - this->timePCD)/getTickFrequency();
                    std::cout << "TIME PCD " << this->timePCD << std::endl;

                }
                else
                {
                    segmentSynth();
                    if(this->useContour.getValue())
                        ContourFromRGBSynth(this->foreground, this->distimage, this->dotimage);
                }

                if (this->saveImages.getValue())
                {
                    cv::Mat* imgseg = new cv::Mat;
                    *imgseg = this->foreground.clone();
                    dataio->listimgseg.push_back(imgseg);
                }
                this->cameraChanged.setValue(false);
            }
        }

        std::cout << "TIME RGBDDATAPROCESSING " << ((double)getTickCount() - timeT)/getTickFrequency() << std::endl;
    }
}



/**
 *        MeshProcessingForKalmanFilter::draw
 * <p>
 *   description:
 *     visualize data
 * @see none
 *
 *  arguments:
 * @param  vparams - a pointer to visualization config
 *
 * @return  none
 *
 */
template <class DataTypes>
void RGBDDataProcessingForKalmanFilter<DataTypes>::draw(const core::visual::VisualParams* vparams)
{
    RGBDDataProcessing<DataTypes>::draw(vparams);
}



/**
 *        RGBDDataProcessingForKalmanFilter::~RGBDDataProcessingForKalmanFilter
 * <p>
 *   description:
 *       destructor
 * @see none
 *
 *  arguments:
 * @param  none
 *
 * @return  none
 *
 */
template <class DataTypes>
RGBDDataProcessingForKalmanFilter<DataTypes>::~RGBDDataProcessingForKalmanFilter()
{

}



}
}
} // namespace sofa

