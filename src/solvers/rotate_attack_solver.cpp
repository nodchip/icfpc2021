#include "stdafx.h"
#include "solver_registry.h"
#include <cmath>

using namespace std;

class RotateAttackSolver : public SolverBase {
public:
  RotateAttackSolver() { }
  SolverParamOut next(const SolverParamIn &gameinfo) override {

    // todo powerが小さいかも
    // todo 符号が間違ってるかも
    Vector center; // 星の座標
    center.x = 0;
    center.y = 0;
    int radius_min2 = 2500; //期待する2乗距離の最小値
    int radius_max2 = 3600; //期待する2乗距離の最大値
    double power = 3.0;
    SolverParamOut ret;
    bool modify_target = true; //相手のvelocityを使って補正する
    for (auto sac : gameinfo.gamestate.shipAndCommands){
      if(sac.ship.role == 1){
	continue;
      }
    
      int radius2 = (sac.ship.position.x - center.x) * (sac.ship.position.x - center.x) + (sac.ship.position.y - center.y) * (sac.ship.position.y - center.y);
      bool clockwise = (sac.ship.position.y - center.y) * (sac.ship.velocity.x) - (sac.ship.position.x - center.x) * (sac.ship.velocity.y) > 0;
      if (radius2 < radius_min2){
	AccelerateCommand cmd;
	cmd.vector.x = int((clockwise ? -power: power) * (sac.ship.position.y - center.y));
	cmd.vector.y = int((clockwise ? power: -power) * (sac.ship.position.x - center.x));
	cmd.type = 0;
	cmd.shipId = sac.ship.shipId;
	ret.appliedCommands.push_back(std::make_shared<AccelerateCommand>(cmd));
	
      }else if(radius2 > radius_max2){
	AccelerateCommand cmd;
	cmd.type = 0;
	cmd.shipId = sac.ship.shipId;
	cmd.vector.x = int((clockwise ? power: -power) * (sac.ship.position.y - center.y));
	cmd.vector.y = int((clockwise ? -power: power) * (sac.ship.position.x - center.x));
	ret.appliedCommands.push_back(std::make_shared<AccelerateCommand>(cmd));
      }else{
	long tgt_min = 11451419;
	Vector target;

	for (auto sac2 : gameinfo.gamestate.shipAndCommands){
	  // 雑に一番近いやつを狙う
	  if(sac2.ship.role == 1){
	    int diffx = sac.ship.position.x - sac2.ship.position.x;
	    int diffy = sac.ship.position.x - sac2.ship.position.x;
	    if(diffx * diffx + diffy * diffy < tgt_min){
	      target.x = sac2.ship.position.x;
	      target.y = sac2.ship.position.y;
	      if (modify_target){
		target.x += sac2.ship.velocity.x;
		target.y += sac2.ship.velocity.y;
	      }
	    }
	  }
	}
	if(tgt_min == 11451419){
	  continue;
	}
	ShootCommand cmd;
	cmd.type = 2;
	cmd.shipId = sac.ship.shipId;
	cmd.target = target;
	ret.appliedCommands.push_back(std::make_shared<ShootCommand>(cmd));
      }
    }
    return ret;
  }
};

REGISTER_SOLVER("RotateAttackSolver", RotateAttackSolver);
