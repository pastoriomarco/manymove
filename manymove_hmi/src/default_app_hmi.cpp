#include "manymove_hmi/default_app_hmi.hpp"

/*  Returns the exact same key list that used to be hard-coded
 *  inside AppModule.  Lambdas now rely only on the `values` map
 *  provided by the engine (merged overrides + current values).       */
std::vector<KeyConfig> DefaultAppModule::buildKeys()
{
    std::vector<KeyConfig> ks;

    /* --- cycle_on_key -------------------------------------------- */
    ks.push_back({"cycle_on_key","bool",true,true,
                  [](const QMap<QString,QString>& v){return v.value("cycle_on_key","");}});

    /* editable ----------------------------------------------------- */
    ks.push_back({"tube_length_key","double",true,true,
                  [](auto& v){return v.value("tube_length_key","");}});
    ks.push_back({"tube_diameter_key","double",true,true,
                  [](auto& v){return v.value("tube_diameter_key","");}});
    ks.push_back({"grasp_offset_key","double",true,true,
                  [](auto& v){return v.value("grasp_offset_key","");}});

    /* computed: tube_scale_key ------------------------------------ */
    ks.push_back({"tube_scale_key","double_array",false,true,
                 [](const QMap<QString,QString>& v)
    {
        const double L = v.value("tube_length_key").toDouble();
        const double D = v.value("tube_diameter_key").toDouble();
        if (L==0.0 || D==0.0) return QString();
        return QString("[%1, %2, %3]").arg(D).arg(D).arg(L);
    }});

    /* pick_post_transform_xyz_rpy_1_key --------------------------- */
    ks.push_back({"pick_post_transform_xyz_rpy_1_key","double_array",false,true,
                 [](const QMap<QString,QString>& v)
    {
        double L = v.value("tube_length_key").toDouble();
        double G = v.value("grasp_offset_key").toDouble();
        if (L==0.0 && G==0.0) return QString();
        double z = (-L/2.0)+G;
        return QString("[%1,%2,%3,3.14,0,0]").arg(0.0).arg(0.0).arg(z);
    }});

    /* insert_post_transform_xyz_rpy_2_key ------------------------- */
    ks.push_back({"insert_post_transform_xyz_rpy_2_key","double_array",false,true,
                 [](const QMap<QString,QString>& v)
    {
        double L=v.value("tube_length_key").toDouble(); if(L==0.0) return QString();
        double z=(-L/2.0);
        return QString("[%1,%2,%3,0,0,-0.785]").arg(0.0).arg(0.0).arg(z);
    }});

    /* load_post_transform_xyz_rpy_2_key --------------------------- */
    ks.push_back({"load_post_transform_xyz_rpy_2_key","double_array",false,true,
                 [](const QMap<QString,QString>& v)
    {
        double L=v.value("tube_length_key").toDouble(); if(L==0.0) return QString();
        double z=-L;
        return QString("[%1,%2,%3,0,0,0]").arg(0.0).arg(0.0).arg(z);
    }});

    /* tube_spawn_pose_key ----------------------------------------- */
    ks.push_back({"tube_spawn_pose_key","pose",false,true,
                 [](const QMap<QString,QString>& v)
    {
        bool ok=false; double L=v.value("tube_length_key").toDouble(&ok);
        if(!ok) return QString();
        double x=(L/2.0)+0.978;  double y=-0.6465; double z=0.8055;
        return QString("{\"x\":%1,\"y\":%2,\"z\":%3,\"roll\":1.57,\"pitch\":2.05,\"yaw\":1.57}")
                      .arg(x).arg(y).arg(z);
    }});

    /* slider_pose_key --------------------------------------------- */
    ks.push_back({"slider_pose_key","pose",false,true,
                 [](const QMap<QString,QString>& v)
    {
        bool ok=false; double L=v.value("tube_length_key").toDouble(&ok);
        if(!ok) return QString();
        double x=L+0.01;
        return QString("{\"x\":%1,\"y\":0.0,\"z\":0.0,\"roll\":0.0,\"pitch\":0.0,\"yaw\":0.0}")
                      .arg(x);
    }});

    return ks;
}
