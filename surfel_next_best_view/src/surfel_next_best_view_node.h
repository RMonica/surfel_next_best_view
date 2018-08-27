/*
 * Copyright (c) 2018, Riccardo Monica
 *   RIMLab, Department of Engineering and Architecture, University of Parma, Italy
 *   http://www.rimlab.ce.unipr.it/
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted
 * provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this list of conditions
 * and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice, this list of
 * conditions and the following disclaimer in the documentation and/or other materials provided with
 * the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors may be used to
 * endorse or promote products derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
 * FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
 * IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef SURFEL_NEXT_BEST_VIEW_NODE_H
#define SURFEL_NEXT_BEST_VIEW_NODE_H

#define PARAM_NAME_EVALUATE_POSES_ACTION "evaluate_poses_action"
#define PARAM_DEFAULT_EVALUATE_POSES_ACTION "evaluate_poses"

#define PARAM_NAME_SET_POINT_CLOUD_ACTION "set_point_cloud_action"
#define PARAM_DEFAULT_SET_POINT_CLOUD_ACTION "set_point_cloud"

#define PARAM_NAME_SET_POINT_CLOUD_TOPIC "set_point_cloud_topic"
#define PARAM_DEFAULT_SET_POINT_CLOUD_TOPIC "point_cloud"

#define PARAM_NAME_FAKE_OPENGL_CONTEXT_SCREEN "fake_opengl_context_screen"
#define PARAM_DEFAULT_FAKE_OPENGL_CONTEXT_SCREEN ""  // leave empty for default

#define PARAM_NAME_FRAME_ID "frame_id"
#define PARAM_DEFAULT_FRAME_ID "/world"

#define PARAM_NAME_ENABLE_ROBOT_FILTER    "enable_robot_filter"
#define PARAM_DEFAULT_ENABLE_ROBOT_FILTER (bool(false))

#define PARAM_NAME_ROBOT_FILTER_PLUGIN    "robot_filter_plugin"
#define PARAM_DEFAULT_ROBOT_FILTER_PLUGIN "RenderRobotURDFPlugin"

#endif // SURFEL_NEXT_BEST_VIEW_NODE_H
