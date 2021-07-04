#include "stdafx.h"
#include "solver_registry.h"

using namespace std;

// 複数の船に司令を送るためにvector
vector<AppliedCommand> solve(const vector<Ship> &ships){
  // 得られた速度に対し其の速度を0にする方向にエンジンをふかすだけ。
  
  // Shipの情報を何らかの方法で受け取る
  vector<AppliedCommand> commands;
  for(auto ship: ships){
    if(ship.role == 0){
      continue;
    }
    if(ship.velocity.x != 0 || ship.velocity.y == 0){
      AccelerateCommand cmd;
      cmd.type = 0;
      cmd.shipId = ship.shipId;
      cmd.vector.x = -ship.velocity.x;
      cmd.vector.y = -ship.velocity.y;
      commands.push_back(cmd);
    }
  }
  return commands;
}

