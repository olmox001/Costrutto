/* SPDX-License-Identifier: GPL-2.0-or-later */
#include "nqg_engine_api.hpp"
#include <cmath>
#include <iostream>
#include <string>
using nqg::api::Host;
static int g_pass=0,g_fail=0;
#define CHECK(c,m) do{if(c){++g_pass;std::cout<<"[PASS] "<<m<<"\n";}else{++g_fail;std::cout<<"[FAIL] "<<m<<"\n";}}while(0)

int main(){
  std::cout<<"=== Capsule vertical dynamics ===\n";
  Host h; h.capsule().renderEnabled=false; h.setViewResolution(48,36);

  auto cycle=[&](double dt, const char* lab){
    h.applyTextCommand("stop");
    h.capsule().freefall=false;
    h.applyTextCommand("tp 0 -3 1.75");
    h.step(dt);
    double z0=h.capsule().observation.position.z;
    double f0=h.capsule().observation.footZ;
    CHECK(std::isfinite(z0)&&z0>0.3, std::string(lab)+" spawn z");
    CHECK(f0>-0.5, std::string(lab)+" feet not deep");
    h.applyTextCommand("jump"); h.step(dt); h.applyTextCommand("jump_release");
    double zmax=z0;
    for(int i=0;i<30;i++){
      h.step(dt);
      double z=h.capsule().observation.position.z;
      double f=h.capsule().observation.footZ;
      CHECK(std::isfinite(z), std::string(lab)+" air finite");
      CHECK(f>-1.0, std::string(lab)+" no deep pen");
      if(z>zmax) zmax=z;
    }
    CHECK(zmax>z0+0.1, std::string(lab)+" gained height");
    for(int i=0;i<40;i++) h.step(dt);
    double z1=h.capsule().observation.position.z;
    double f1=h.capsule().observation.footZ;
    CHECK(std::isfinite(z1), std::string(lab)+" land finite");
    CHECK(f1>-0.5, std::string(lab)+" land feet");
  };
  cycle(1./60, "dt60");
  cycle(1./30, "dt30");

  h.applyTextCommand("tp 0 -2 1.75"); h.step(1./60);
  h.applyTextCommand("jump"); h.step(1./60); h.applyTextCommand("jump_release");
  for(int i=0;i<50;i++){
    h.step(1./60);
    auto&o=h.capsule().observation;
    CHECK(std::isfinite(o.position.z),"ceil finite");
    CHECK(o.position.z<8.0,"no tunnel");
    CHECK(o.footZ>-2.0,"no deep under");
  }
  std::cout<<"RESULT: "<<g_pass<<" PASS, "<<g_fail<<" FAIL\n";
  return g_fail?1:0;
}
