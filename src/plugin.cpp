#include <gazebo/gazebo.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Collision.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Shape.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/rendering/Conversions.hh>
#include <OGRE/OgreAxisAlignedBox.h>
#include <gazebo/rendering/rendering.hh>
#include <gazebo/rendering/Visual.hh>
#include <ignition/math.hh>
#include <iostream>
#include <ros/console.h>
#include <ros/assert.h>
#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>

//必须的头文件，而gazebo/physics/physics.hh, gazebo/rendering/rendering.hh, or gazebo/sensors/sensors.hh 按需添加
//所有插件都要在gazebo名称空间下
namespace gazebo {                                                    //每种插件都必须从特定类型的插件类型中继承
    class DumpGTBox : public WorldPlugin {
    public:
        DumpGTBox() : WorldPlugin() {
            printf("DumpGTBox constructed!\n");
        }

    public:
        void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf) {
            // save handle
            world_ptr = _world;
            scene_ptr = gazebo::rendering::get_scene();

            if (!scene_ptr) {
                ROS_WARN("no camera or other rendering object in this world.\n"
                         "try to add model://camera at origin by default");
                world_ptr->InsertModelFile("model://camera");
            }

            // read input parameter
            {
                if (_sdf->HasElement("save_dir")) {
                    this->save_dir = _sdf->Get<std::string>("save_dir");
                    ROS_INFO(ROS_PACKAGE_NAME " <save_dir>: %s", this->save_dir.c_str());
                } else {
                    this->save_dir = "~/Documents/dump_dt_box";
                    ROS_INFO(ROS_PACKAGE_NAME " <save_dir>: default to %s", this->save_dir.c_str());
                }

                if (_sdf->HasElement("model_name_to_dump")) {
                    std::string temp = _sdf->Get<std::string>("model_name_to_dump");
                    boost::split(model_name_to_dump, temp,
                                 boost::is_any_of(",/"),
                                 boost::token_compress_on);
                    std::stringstream ss;
                    for (const auto name:model_name_to_dump) {
                        ss << name << " ";
                    }
                    ROS_INFO(ROS_PACKAGE_NAME " <model_name_to_dump>: %s", ss.str().c_str());
                } else {
                    this->model_name_to_dump.push_back("dump_all_model_gt_box");
                    ROS_INFO(ROS_PACKAGE_NAME " <model_name_to_dump>: default to %s", "**ALL**");
                }
            }

            // create dir to dump
            {
                std::stringstream sss;
                sss << "mkdir -p " << save_dir;
                system(sss.str().c_str());
            }

            this->updateConnection = event::Events::ConnectWorldUpdateBegin(
                    std::bind(&DumpGTBox::OnUpdate, this));
        }

        void OnUpdate() {
            //  search model by name
            cnt++;

            // 是否已获取scene场景信息
            {
                if (scene_ptr == NULL) {
                    ROS_WARN("rendering has been created. try to get scene handle once again ...");
                    scene_ptr = gazebo::rendering::get_scene();
                    if (scene_ptr) {
                        ROS_WARN("get scene handle! begin to search bbox");
                    } else {
                        return;
                    }
                }
            }

            // 更新 bbox
            if (cnt % 1000 == 1) {
                auto model_num = world_ptr->ModelCount();
                bool dump_all = model_name_to_dump[0] == "dump_all_model_gt_box" ? true : false;
                std::ofstream file(save_dir + "/label" + std::to_string((int) (cnt / 1000)) + ".txt");

                ROS_INFO(ROS_PACKAGE_NAME" > model_num[%d]", model_num);
                file << (__gnu_cxx::__to_xstring<std::string>)
                        (&std::vsnprintf, 400,
                         "%-30s %-30s"
                         " %15s "
                         " %15s "
                         " %15s ",
                         "gazebo model name",
                         "name to dump",
                         "dx dy dz", "position", "rpy"
                        ) << std::endl;
                for (int i = 0; i < model_num; ++i) {
                    physics::ModelPtr this_model_ptr = world_ptr->ModelByIndex(i);
                    if (!this_model_ptr) continue;
                    std::string this_model_name = this_model_ptr->GetName();

                    for (int idx_in_list = 0; idx_in_list < model_name_to_dump.size(); idx_in_list++) {
                        if (this_model_name.find(model_name_to_dump[idx_in_list]) != std::string::npos || dump_all) {
                            rendering::VisualPtr vis = scene_ptr->GetVisual(this_model_name);
                            if (vis) {
                                auto label = make_label(vis);
                                auto temp = dump_all ? this_model_name : model_name_to_dump[idx_in_list];
                                file << (__gnu_cxx::__to_xstring<std::string>)
                                        (&std::vsnprintf, 400,
                                         "%-30s %-30s"
                                         "  %5.3lf %5.3lf %5.3lf  "
                                         "  %5.3lf %5.3lf %5.3lf  "
                                         "  %5.3lf %5.3lf %5.3lf  ",
                                         this_model_name.c_str(),
                                         temp.c_str(),
                                         label[8].x, label[8].y, label[8].z,
                                         label[9].x, label[9].y, label[9].z,
                                         label[10].x, label[10].y, label[10].z
                                        ) << std::endl;
                            }
                            break;
                        }
                    }
                }
                ROS_INFO(ROS_PACKAGE_NAME " dump to file completely");
                file.close();
            }
        }

        std::vector<Ogre::Vector3> make_label(rendering::VisualPtr vis) {

            ignition::math::Box box = vis->BoundingBox(); // 真离谱啊。得到世界坐标系下粗暴的3dbbox : auto box = model->CollisionBoundingBox();

            // 获取8个角点。
            /*
                       |y+
                       |
                       1-------2
                      /|      /|
                     / |     / |
                    5-------4  |
                    |  0----|--3 ———— x+
                    | /     | /
                    |/      |/
                    6-------7
                   /
                  z+
            */
            Ogre::AxisAlignedBox box_Ogre(rendering::Conversions::Convert(box.Min()),
                                          rendering::Conversions::Convert(box.Max()));
            Ogre::Vector3 size = box_Ogre.getSize();
            auto corners = box_Ogre.getAllCorners();
            std::vector<Ogre::Vector3> corners_world(8 + 3); // corners8,size,centroid,rotation

            // local to world
            Ogre::Matrix4 T_w2o(rendering::Conversions::Convert(vis->Pose().Rot()));

            // map to world frame
            T_w2o.setTrans(rendering::Conversions::Convert(vis->Pose().Pos()));
            Ogre::Vector3 centroid(0, 0, 0);
            for (int i = 0; i < 8; i++) {
                corners_world[i] = T_w2o * corners[i];
                centroid += corners_world[i];
            }
            corners_world[8] = size;
            corners_world[9] = centroid / 8.0;
            corners_world[10] = Ogre::Vector3(vis->Rotation().Roll(), vis->Rotation().Pitch(), vis->Rotation().Yaw());
            return corners_world;
        }

        physics::WorldPtr world_ptr;
        rendering::ScenePtr scene_ptr;

        std::string save_dir;
        std::vector<std::string> model_name_to_dump;

        event::ConnectionPtr updateConnection;
        int cnt = 0;
    };

    GZ_REGISTER_WORLD_PLUGIN(DumpGTBox)
}