#include <iostream>
#include <fstream>
#include <string>
#include "cv.h"
#include "highgui.h"

using namespace std;

int main()
 {
	ifstream fin("calibdata.txt"); /* ��������ͼ���ļ���·�� */
	string filename;

	int image_count=0;  /* ͼ������ */
	CvSize image_size;  /* ͼ��ĳߴ� */
	CvSize board_size = cvSize(5,7);    /* �������ÿ�С��еĽǵ��� */
	int total_per_image = board_size.width*board_size.height;
	int step;
	int successes=0;
	int count;
	CvPoint2D32f* image_points_buf = new CvPoint2D32f[total_per_image];   /* ����ÿ��ͼ���ϼ�⵽�Ľǵ� */

	char names[200];
	FILE *fp = fopen("calibdata.txt", "r");
	/*ͳ�ƶ���ͼƬ����*/
	while (fscanf(fp,"%s ",names)==1)
	{
	    ++image_count;
	}

    CvMat* object_points = cvCreateMat(total_per_image*image_count,3,CV_32FC1); /* ���涨����Ͻǵ����ά���� */
	CvMat* image_points = cvCreateMat(total_per_image*image_count,2,CV_32FC1); /* ������ȡ�����нǵ� */
	CvMat* point_counts = cvCreateMat(image_count,1,CV_32SC1); /* ÿ��ͼ���нǵ������ */
	CvMat* intrinsic_matrix = cvCreateMat(3,3,CV_32FC1); /* ������ڲ������� */
	CvMat* distortion_coeffs = cvCreateMat(1,4,CV_32FC1); /* �������4������ϵ����k1,k2,p1,p2 */

    /************************************************************************
	       ��ȡÿһ��ͼ�񣬴�����ȡ���ǵ㣬Ȼ��Խǵ���������ؾ�ȷ��
	*************************************************************************/

   cout<<"��ʼ��ȡ�ǵ㡭����������";

   for(int num = 1;num <= image_count;num++)
    {
        cout<<"\n ��ȡ��"<<" "<<num<<" "<<"��ͼ�� ������ƶ���ͼ�����ڴ��� ������س�������һ��ͼ��Ľǵ���ȡ \n";
		getline(fin,filename);
		IplImage* img = cvLoadImage(filename.c_str(), 1);
		image_size.width = img->width;
        image_size.height = img->height;
		/* ��ȡ�ǵ� */
		if (0 == cvFindChessboardCorners( img, board_size,
            image_points_buf, &count, CV_CALIB_CB_ADAPTIVE_THRESH ))
		{
			cout<<"can not find chessboard corners!\n";
			exit(1);
		} else {
			IplImage* img_gray = cvCreateImage(cvGetSize(img),8,1);
			cvCvtColor( img, img_gray, CV_BGR2GRAY );
			/* �����ؾ�ȷ�� */
			cvFindCornerSubPix( img_gray, image_points_buf, count, cvSize(11,11),
				cvSize(-1,-1), cvTermCriteria( CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 30, 0.1 ));
			/* ��ͼ������ʾ�ǵ�λ�� */
			cvDrawChessboardCorners( img, board_size, image_points_buf, count, 1);
			cvShowImage("calib",img);
			static int images_num=1;
			char filename[200];
			sprintf(filename,"corner_img\\%d.jpg" ,images_num++);
			cvSaveImage(filename,img);
			cvWaitKey();
			cvReleaseImage(&img);
		}
		 if(total_per_image == count)
            {
                step=successes*total_per_image;

                for(int i=step,j=0;j<total_per_image;++i,++j)
                {
                    CV_MAT_ELEM(*image_points,float,i,0)=image_points_buf[j].x;
                    CV_MAT_ELEM(*image_points,float,i,1)=image_points_buf[j].y;
                    CV_MAT_ELEM(*object_points,float,i,0)=(float)(j/board_size.width);
                    CV_MAT_ELEM(*object_points,float,i,1)=(float)(j%board_size.width);
                    CV_MAT_ELEM(*object_points,float,i,2)=0.0f;
                }
                CV_MAT_ELEM(*point_counts,int,successes,0)=total_per_image;
                successes++;
            }
	}

	cout<<"\n�ǵ���ȡ��ɣ�\n";
	cout<< "\n����ȡ"<<" "<<image_count<<" "<<"��ͼ�񣬹��ɹ�"<<" "<<successes<<" "<<"��\n";

	/************************************************************************
	       ���������
	*************************************************************************/
	cout<<"\n��ʼ���ꡭ����������\n";

	/* ��ʼ��������Ͻǵ����ά���� */

	CvMat * object_points2=cvCreateMat(successes*total_per_image,3,CV_32FC1);
	CvMat * image_points2=cvCreateMat(successes*total_per_image,2,CV_32FC1);
	CvMat * point_counts2=cvCreateMat(successes,1,CV_32SC1);

	for(int i=0;i<successes*total_per_image;++i){
		CV_MAT_ELEM(*image_points2,float,i,0)=CV_MAT_ELEM(*image_points,float,i,0);
		CV_MAT_ELEM(*image_points2,float,i,1)=CV_MAT_ELEM(*image_points,float,i,1);
		CV_MAT_ELEM(*object_points2,float,i,0)=CV_MAT_ELEM(*object_points,float,i,0);
		CV_MAT_ELEM(*object_points2,float,i,1)=CV_MAT_ELEM(*object_points,float,i,1);
		CV_MAT_ELEM(*object_points2,float,i,2)=CV_MAT_ELEM(*object_points,float,i,2);
	}

	for(int i=0;i<successes;++i){
		CV_MAT_ELEM(*point_counts2,int,i,0)=CV_MAT_ELEM(*point_counts,int,i,0);
	}

	cvReleaseMat(&object_points);
	cvReleaseMat(&image_points);
	cvReleaseMat(&point_counts);

	CV_MAT_ELEM(*intrinsic_matrix,float,0,0)=1.0f;
	CV_MAT_ELEM(*intrinsic_matrix,float,1,1)=1.0f;

	CvMat* rotation_vectors = cvCreateMat(successes,3,CV_32FC1);/* ÿ��ͼ�����ת���� */
	CvMat* translation_vectors = cvCreateMat(successes,3,CV_32FC1);/* ÿ��ͼ���ƽ������ */
	/* ��ʼ���� */
    cvCalibrateCamera2(object_points2,
					   image_points2,
                       point_counts2,
					   image_size,
                       intrinsic_matrix,
					   distortion_coeffs,
                       rotation_vectors,
					   translation_vectors,
					   0);

	cout<<"\n������ɣ�\n\n";
	cout<<CV_MAT_ELEM(*intrinsic_matrix,float,0,0)<<"    "<<CV_MAT_ELEM(*intrinsic_matrix,float,0,1)
															<<"    "<<CV_MAT_ELEM(*intrinsic_matrix,float,0,2)
															<<"\n\n";
	cout<<CV_MAT_ELEM(*intrinsic_matrix,float,1,0)<<"    "<<CV_MAT_ELEM(*intrinsic_matrix,float,1,1)
															<<"    "<<CV_MAT_ELEM(*intrinsic_matrix,float,1,2)
															<<"\n\n";
	cout<<CV_MAT_ELEM(*intrinsic_matrix,float,2,0)<<"    "<<CV_MAT_ELEM(*intrinsic_matrix,float,2,1)
															<<"          "<<CV_MAT_ELEM(*intrinsic_matrix,float,2,2)
															<<"\n\n";
    /*����궨���*/
    cvSave("calib_index\\Intrinsics.xml",intrinsic_matrix);//�ڲ�������
	cvSave("calib_index\\Distortion.xml",distortion_coeffs);//����ϵ��
	cvSave("calib_index\\rotation.xml",rotation_vectors);//��ת����
	cvSave("calib_index\\translation.xml",translation_vectors);//ƽ������

	cvReleaseMat(&object_points2);
	cvReleaseMat(&image_points2);
	cvReleaseMat(&point_counts2);
	cvReleaseMat(&intrinsic_matrix);
	cvReleaseMat(&distortion_coeffs);
	cvReleaseMat(&rotation_vectors);
	cvReleaseMat(&translation_vectors);

	return 0;
}
