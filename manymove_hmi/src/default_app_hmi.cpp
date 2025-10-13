// Copyright 2025 Flexin Group SRL
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the Flexin Group SRL nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include "manymove_hmi/default_app_hmi.hpp"

/*  Returns the exact same key list that used to be hard-coded
 *  inside AppModule.  Lambdas now rely only on the `values` map
 *  provided by the engine (merged overrides + current values).       */
std::vector<KeyConfig> DefaultAppModule::buildKeys()
{
  std::vector<KeyConfig> ks;

  /* --- cycle_on_key -------------------------------------------- */
  ks.push_back(
    {"cycle_on_key", "bool", true, true,
      [](const QMap<QString, QString> & v) {return v.value("cycle_on_key", "");}, 1.0, "", false,
      0});

  /* editable ----------------------------------------------------- */
  ks.push_back(
    {"tube_length_key", "double", true, true,
      [](auto & v) {return v.value("tube_length_key", "");}, 1000.0, "mm", true, 1000, 47.0,
      150.0});
  ks.push_back(
    {"tube_diameter_key", "double", true, true,
      [](auto & v) {return v.value("tube_diameter_key", "");}, 1000.0, "mm", true, 1000, 10.0,
      16.0});
  ks.push_back(
    {"grasp_offset_key", "double", true, true,
      [](auto & v) {return v.value("grasp_offset_key", "");}, 1000.0, "mm", true, 1000, 20.0,
      30.0});

  /* computed: tube_scale_key ------------------------------------ */
  ks.push_back(
    {"tube_scale_key", "double_array", false, false, [](const QMap<QString, QString> & v) {
        const double L = v.value("tube_length_key").toDouble();
        const double D = v.value("tube_diameter_key").toDouble();
        if (L == 0.0 || D == 0.0) {
          return QString();
        }
        return QString("[%1, %2, %3]").arg(D).arg(D).arg(L);
      }});

  /* pick_post_transform_xyz_rpy_1_key --------------------------- */
  ks.push_back(
    {"pick_post_transform_xyz_rpy_1_key", "double_array", false, false,
      [](const QMap<QString, QString> & v) {
        double L = v.value("tube_length_key").toDouble();
        double G = v.value("grasp_offset_key").toDouble();
        if (L == 0.0 && G == 0.0) {
          return QString();
        }
        double z = (-L / 2.0) + G;
        return QString("[%1,%2,%3,3.14,0,0]").arg(0.0).arg(0.0).arg(z);
      }});

  /* insert_post_transform_xyz_rpy_2_key ------------------------- */
  ks.push_back(
    {"insert_post_transform_xyz_rpy_2_key", "double_array", false, false,
      [](const QMap<QString, QString> & v) {
        double L = v.value("tube_length_key").toDouble();
        if (L == 0.0) {
          return QString();
        }
        double z = (-L / 2.0);
        return QString("[%1,%2,%3,0,0,-0.785]").arg(0.0).arg(0.0).arg(z);
      }});

  /* load_post_transform_xyz_rpy_2_key --------------------------- */
  ks.push_back(
    {"load_post_transform_xyz_rpy_2_key", "double_array", false, false,
      [](const QMap<QString, QString> & v) {
        double L = v.value("tube_length_key").toDouble();
        if (L == 0.0) {
          return QString();
        }
        double z = -L;
        return QString("[%1,%2,%3,0,0,0]").arg(0.0).arg(0.0).arg(z);
      }});

  /* tube_spawn_pose_key ----------------------------------------- */
  ks.push_back(
    {"tube_spawn_pose_key", "pose", false, false, [](const QMap<QString, QString> & v) {
        bool ok = false;
        double L = v.value("tube_length_key").toDouble(&ok);
        if (!ok) {
          return QString();
        }
        double x = (L / 2.0) + 0.978;
        double y = -0.6465;
        double z = 0.8055;
        return QString(
          "{\"x\":%1,\"y\":%2,\"z\":%3,\"roll\":1.57,\"pitch\":2.05,\"yaw\":1.57}").arg(x).arg(
          y).arg(z);
      }});

  /* slider_pose_key --------------------------------------------- */
  ks.push_back(
    {"slider_pose_key", "pose", false, false, [](const QMap<QString, QString> & v) {
        bool ok = false;
        double L = v.value("tube_length_key").toDouble(&ok);
        if (!ok) {
          return QString();
        }
        double x = L + 0.01;
        return QString(
          "{\"x\":%1,\"y\":0.0,\"z\":0.0,\"roll\":0.0,\"pitch\":0.0,\"yaw\":0.0}").arg(x);
      }});

  return ks;
}
