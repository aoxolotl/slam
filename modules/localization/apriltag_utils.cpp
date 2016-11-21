#include "apriltag_utils.h"
#include <math.h>
#define SQ(a) (a * a)

void apriltag_init(apriltag_detector *td, apriltag_family *tf,
	float decimate, float blur, int num_threads, 
	int debug, int ref_edg, int ref_dec, int ref_pose)  
{
    apriltag_detector_add_family(td, tf);
    td->quad_decimate = decimate;
    td->quad_sigma = blur; 
    td->nthreads = num_threads;
    td->debug = debug;
    td->refine_edges = ref_edg;
    td->refine_decode = ref_dec;
    td->refine_pose = ref_pose;
}

void getExtrinsics(apriltag_detection *det, double fx, double fy,
					double *rot, double *trans)
{
	double *homography = det->H->data;

	double scale_factor, sf1, sf2;

	sf1 = sqrt(SQ(homography[0] / fx) 
				+ SQ(homography[3] / fy)
				+ SQ(homography[6]));


	sf2 = sqrt(SQ(homography[1] / fx) 
				+ SQ(homography[4] / fy)
				+ SQ(homography[7]));

	scale_factor = sqrt(sf1 * sf2);

	rot[0] = homography[0] / (scale_factor * fx);
	rot[1] = homography[3] / (scale_factor * fy);
	rot[2] = homography[6] / scale_factor;

	rot[3] = homography[1] / (scale_factor * fx);
	rot[4] = homography[4] / (scale_factor * fy);
	rot[5] = homography[7] / scale_factor;

	rot[6] = rot[1] * rot[5] - (rot[4] * rot[2]);
	rot[7] = rot[3] * rot[2] - (rot[0] * rot[5]);
	rot[8] = rot[0] * rot[4] - (rot[3] * rot[1]);

	trans[0] = homography[2] / (scale_factor * fx);
	trans[1] = homography[5] / (scale_factor * fy);
	trans[2] = homography[8] / scale_factor;
}

void getEulerAngles(double *r, double *ang_x, double *ang_y, double *ang_z)
{
	*ang_x = atan2(r[7], r[8]);
	*ang_y = atan2(-r[6], sqrt(SQ(r[0]) + SQ(r[3])));
	*ang_z = atan2(r[3], r[0]);
}
