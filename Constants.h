/* 
 * File:   Constants.h
 * Author: macbook
 *
 * Created on 16 Апрель 2013 г., 16:58
 */

#ifndef CONSTANTS_H
#define	CONSTANTS_H

class Constants {
public:
    static const int min_win;
    static const int patch_size;
    static const double ncc_thesame;
    static const double valid;
    static const int num_trees;
    static const int num_features;
    static const double thr_fern ;
    static const double thr_nn;
    static const double thr_nn_valid;
    static const int num_closest_init;
    static const int num_warps_init;
    static const int noise_init;
    static const int angle_init;
    static const double shift_init;
    static const double scale_init;
    static const int num_closest_update;
    static const int num_warps_update;
    static const int noise_update;
    static const int angle_update;
    static const double shift_update;
    static const double scale_update;
    static const double overlap;
    static const int num_patches;
    static const int bb_x;
    static const int bb_y;
    static const int bb_w;
    static const int bb_h;
private:

};

#endif	/* CONSTANTS_H */

