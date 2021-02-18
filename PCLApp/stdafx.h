
// stdafx.h : 标准系统包含文件的包含文件，
// 或是经常使用但不常更改的
// 特定于项目的包含文件

#pragma once

#ifndef VC_EXTRALEAN
#define VC_EXTRALEAN            // 从 Windows 头中排除极少使用的资料
#endif

#include "targetver.h"

#define _ATL_CSTRING_EXPLICIT_CONSTRUCTORS      // 某些 CString 构造函数将是显式的

// 关闭 MFC 对某些常见但经常可放心忽略的警告消息的隐藏
#define _AFX_ALL_WARNINGS

#include <afxwin.h>         // MFC 核心组件和标准组件
#include <afxext.h>         // MFC 扩展





#ifndef _AFX_NO_OLE_SUPPORT
#include <afxdtctl.h>           // MFC 对 Internet Explorer 4 公共控件的支持
#endif
#ifndef _AFX_NO_AFXCMN_SUPPORT
#include <afxcmn.h>             // MFC 对 Windows 公共控件的支持
#endif // _AFX_NO_AFXCMN_SUPPORT

#include <afxcontrolbars.h>     // 功能区和控件条的 MFC 支持


#include <afxsock.h>            // MFC 套接字扩展

#undef min 
#undef max 
#define DEBUG_NEW new

#define WM_NMDBLCLK            WM_USER+4
#include <DbgHelp.h>
#include <hidsdi.h>
#include <setupapi.h>
#include "StaticFunction.h"

#include <vtkAutoInit.h>
VTK_MODULE_INIT(vtkRenderingOpenGL2);
VTK_MODULE_INIT(vtkInteractionStyle);
VTK_MODULE_INIT(vtkRenderingFreeType);
//#define vtkRenderingCore_AUTOINIT 2(vtkInteractionStyle,vtkRenderingOpenGL2)

#include <vtk_glew.h>
#include <vtkObject.h>
#include <vtkVersion.h>
#include <vtkActor.h>
#include <vtkImageActor.h>
#include <vtkBMPReader.h>
#include <vtkPNGReader.h>
#include <vtkSTLReader.h>
#include <vtkQuadric.h> 
#include <vtkSampleFunction.h> 
#include <vtkShrinkFilter.h> 
#include <vtkSmartPointer.h>
#include <vtkOrientationMarkerWidget.h>
#include <vtkBooleanOperationPolyDataFilter.h>
#include <vtkTextProperty.h>
#include <vtkCamera.h>
#include <vtkCubeAxesActor2D.h>
#include <vtkAnnotatedCubeActor.h>
#include <vtkExtractGeometry.h> 
#include <vtkAxesActor.h>
#include <vtkPLYReader.h>
#include <vtkOBJReader.h>
#include <vtkTriangle.h>
#include <vtkTriangleFilter.h>
#include <vtkPolyDataMapper.h>
#include <vtkMassProperties.h>
#include <vtkCylinder.h>
#include <vtkLine.h>
#include <vtkPlane.h>
#include <vtkSphere.h>
#include <vtkImplicitBoolean.h>
#include <vtkCylinderSource.h>
#include <vtkCubeSource.h>
#include <vtkSphereSource.h>
#include <vtkProperty.h>
#include <vtkClipPolyData.h>
#include <vtkTransformPolyDataFilter.h>
#include <vtkTransform.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkRenderWindow.h>
#include <vtkSmartPointer.h>
#include <vtkWorldPointPicker.h>
#include <vtkWin32OpenGLRenderWindow.h>
#include <vtkObjectFactory.h>
#include <vtkRenderer.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkMath.h>
#include <vtkNamedColors.h>
#include <vtkPropPicker.h>
#include <vtkInteractorObserver.h>
#include <vtkCommand.h>
#include <vtkBoxWidget.h>
#include <vtkProp3DCollection.h>
#include <vtkCaptionActor2D.h>
#include <vtkTextActor.h>
#include <vtkTextProperty.h>
#include <vtkCaptionWidgetPositionCallback.h>
#include <vtkInteractorStyleTrackballActor.h>
#include <vtkInteractorStyleRubberBandPick.h>
#include <vtkRendererCollection.h>
//#include <vtkoutputwindow.h>

#include <boost/thread/thread.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/io/openni2_grabber.h>
#include <pcl/io/openni2/openni.h>
//#include <Eigen/src/Core/util/StaticAssert.h>
#include <pcl/io/openni2/openni2_device_info.h>
//#include <OpenNI.h>
#include <pcl/octree/octree.h>
#include<pcl/common/eigen.h>
//#include<Eigen/Geometry.h>

#include <pcl/octree/octree_pointcloud_changedetector.h>
#include <pcl/compression/octree_pointcloud_compression.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/point_representation.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/surface/gp3.h>
#include <pcl/console/parse.h>
#include <pcl/conversions.h>
#include <pcl/common/common_headers.h>
#include <pcl/common/transforms.h>
#include <pcl/correspondence.h>
#include "pcl/visualization/pcl_visualizer.h"
#include <pcl/visualization/interactor_style.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/range_image_visualizer.h> 
#include <pcl/visualization/image_viewer.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/shot_omp.h>
#include <pcl/features/board.h>
#include <pcl/features/boundary.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/features/impl/fpfh.hpp>
#include <pcl/features/integral_image_normal.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/features/range_image_border_extractor.h>
#include <pcl/features/narf_descriptor.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/crop_hull.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/filters/approximate_voxel_grid.h>   //滤波类头文件  （使用体素网格过滤器处理的效果比较好）
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/keypoints/narf_keypoint.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
//#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_circle.h>
#include <pcl/sample_consensus/sac_model_circle3d.h>
#include <pcl/segmentation/region_growing_rgb.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/min_cut_segmentation.h>
#include <pcl/surface/mls.h>
#include <pcl/registration/ndt.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/recognition/cg/hough_3d.h>
#include <pcl/recognition/cg/geometric_consistency.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_distance.h>
#include <pcl/recognition/implicit_shape_model.h>
//#include <pcl/recognition/impl/implicit_shape_model.hpp>

#include <pcl/range_image/range_image.h>



#define RGB(r,g,b)          ((COLORREF)(((BYTE)(r)|((WORD)((BYTE)(g))<<8))|(((DWORD)(BYTE)(b))<<16)))

/************Glog******************/
#include <glog/logging.h>



/************Ranger3SDK**********/
#include <opencv2/opencv.hpp>
#include <opencv2\imgproc\types_c.h>
#include <Ranger3.h>



#include "Mmsystem.h"
#include<vector>

#include "tinyxml2.h"
#include "dbt.h"
//#include "hidapi.h"
#include <GoSdk/GoSdk.h>
//#include "EthernetScanner3D.h"
#include "EthernetScanner3DDefine.h"
/************FocalSpec**********/
#include "PeakStructure.h"
#include "CameraDll.h"
#include "CameraStatus.h"
#include "Callback.h"
#include "VevoParameterDefinitions.h"
/*******************************/
#include "EthernetScannerSDK.h"
#include "EthernetScannerSDKDefine.h"

/************PolyWorksSDK**********/
#include "PolyWorksSDK/COM/IIMInspect.h"
#include "PolyWorksSDK/COM/IIMPointCloud.h"
#include "PolyWorksSDK/COM/IIMSceneCamera.h"
//#include "PolyWorksSDK/COM/IIMInspect_i.c"
//#include "PolyWorksSDK/COM/IIMPointCloud_i.c"
//#include "PolyWorksSDK/COM/IIMSceneCamera_i.c"
//#include "PolyWorksSDK/COM/IMInspect_i.c"



/*****************VisionaryT******************************/
#include <boost/smart_ptr/shared_ptr.hpp>
#include <boost/smart_ptr/make_shared.hpp>
#include "VisionaryAutoIPScan.h"
#include "VisionaryTData.h"    // Header specific for the Time of Flight data
#include "VisionaryDataStream.h"
#include "VisionaryControl.h"
#include "PointCloudPlyWriter.h"
#include "CoLaBCommandBuilder.h"
#include "CoLaBCommandReader.h"



