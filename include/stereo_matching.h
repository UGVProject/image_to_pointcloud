//
// Created by zh on 23/5/17.
//

#ifndef IMAGE_TO_POINTCLOUD_STEREO_MATCHING_H
#define IMAGE_TO_POINTCLOUD_STEREO_MATCHING_H
#include <VX/vx.h>

class StereoMatching
{
public:

    enum ImplementationType
    {
        HIGH_LEVEL_API,
        LOW_LEVEL_API,
        LOW_LEVEL_API_PYRAMIDAL
    };

    struct StereoMatchingParams
    {
        // disparity range
        vx_int32 min_disparity;
        vx_int32 max_disparity;

        // discontinuity penalties
        vx_int32 P1;
        vx_int32 P2;

        // SAD window size
        vx_int32 sad;

        // Census Transform window size
        vx_int32 ct_win_size;

        // Hamming cost window size
        vx_int32 hc_win_size;

        // BT-cost clip value
        vx_int32 bt_clip_value;

        // validation threshold
        vx_int32 max_diff; // cross-check
        vx_int32 uniqueness_ratio;

        vx_enum scanlines_mask;

        vx_enum flags;

        StereoMatchingParams();
    };

    static StereoMatching* createStereoMatching(vx_context context, const StereoMatchingParams& params,
                                                ImplementationType impl,
                                                vx_image left, vx_image right, vx_image disparity);

    virtual ~StereoMatching() {}

    virtual void run() = 0;

    virtual void printPerfs() const = 0;
};

#endif //IMAGE_TO_POINTCLOUD_STEREO_MATCHING_H
