#pragma once
#include <vector>
#include <memory>

class Vector {
public:
  int64_t x = 0;
  int64_t y = 0;
};

class AppliedCommand {
public:
  using Ptr = std::shared_ptr<AppliedCommand>;
  virtual ~AppliedCommand() {}
  int64_t type = -1;
  int64_t shipId = -1;
};

class AccelerateCommand : public AppliedCommand {
public:
  virtual ~AccelerateCommand() {}
  Vector vector;
};

class DetonateCommand : public AppliedCommand {
public:
  virtual ~DetonateCommand() {}
};

class ShootCommand : public AppliedCommand {
public:
  virtual ~ShootCommand() {}
  Vector target;
  int64_t x3;
};

class Ship {
public:
  int64_t role = -1;
  int64_t shipId = -1;
  Vector  position;
  Vector velocity;
  int64_t x4 = -1;
  int64_t x5 = -1;
  int64_t x6 = -1;
  int64_t x7 = -1;
};

class ShipAndCommands {
public:
  Ship ship;
  std::vector<std::shared_ptr<AppliedCommand>> appliedCommands;
};

class GameState {
public:
  int64_t gameTick = -1;
  int64_t x1 = -1;
  std::vector<ShipAndCommands> shipAndCommands;
};

class GameInfo {
public:
  int64_t x0 = -1;
  int64_t role = -1;
  int64_t x2 = -1;
  int64_t x3 = -1;
  int64_t x4 = -1;
};

class GameResponse {
public:
  int64_t gameStage = -1;
  GameInfo gameInfo;
  GameState gameState;
};
