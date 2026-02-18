export interface RobotPose {
  x: number;
  y: number;
  theta: number;
}

export interface NavigationGoal {
  robotId: string;
  x: number;
  y: number;
  theta: number;
}
