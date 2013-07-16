/*
 * Software License Agreement (Modified BSD License)
 *
 *  Copyright (c) 2012, PAL Robotics, S.L.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of PAL Robotics, S.L. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * @file trackerinterface.h
 * @author Bence Magyar
 * @date March 2012
 * @version 0.1
 * @brief This class describes an interface derived from the blort tracker as a complete application.
 * It defines the system logic of recovery and track modes and confidence states.
 * @namespace pal_blort
 */

#ifndef BLORTTRACKER_H
#define BLORTTRACKER_H

#include <opencv2/core/core.hpp>
#include <ros/console.h> //TODO: remove

namespace blort_ros
{
    enum tracker_mode
    {
        TRACKER_RECOVERY_MODE,
        TRACKER_TRACKING_MODE,
        TRACKER_LOCKED_MODE
    };

    enum tracker_confidence
    {
        TRACKER_CONF_GOOD,
        TRACKER_CONF_FAIR,
        TRACKER_CONF_LOST
    };

    class TrackerInterface
    {
    protected:
        tracker_mode current_mode;
        tracker_confidence current_conf;
        cv::Mat last_image;

    public:
        TrackerInterface()
        {
            current_mode = TRACKER_RECOVERY_MODE;
            current_conf = TRACKER_CONF_LOST;
        }
        virtual void track() = 0;
        virtual void recovery() = 0;
        void process(cv::Mat img)
        {
            last_image       = img;            
            switch(current_mode)
            {
            case TRACKER_RECOVERY_MODE:
                recovery();
                break;
            case TRACKER_TRACKING_MODE:
                track();
                break;
            case TRACKER_LOCKED_MODE:
                track(); //FIXME a bit hacky
                break;
            }
        }
        virtual void switchToTracking(){ current_conf = TRACKER_CONF_FAIR; current_mode = TRACKER_TRACKING_MODE; }
        virtual void switchToRecovery(){ current_conf = TRACKER_CONF_LOST; current_mode = TRACKER_RECOVERY_MODE; }
        virtual void reset(){ switchToRecovery(); }
        tracker_mode getMode(){ return current_mode; }
        tracker_confidence getConfidence(){ return current_conf; }
    };
}

#endif // BLORTTRACKER_H
