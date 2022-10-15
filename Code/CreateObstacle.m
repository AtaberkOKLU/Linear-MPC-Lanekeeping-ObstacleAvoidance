function [f_x_TV, v_x_TV, TV_x] = CreateObstacle(Obstacles, T_sampling)
    f_x_TV  = [];
    v_x_TV  = [];
    TV_x    = cell(size(Obstacles));
    for obs = 1:length(Obstacles)
        obstacle = Obstacles(obs);
        if mean(obstacle.width) < 0
            f_x_TV = [f_x_TV;0 0 0 -1 0]; % e_y
            v_x_TV = [v_x_TV;-max(obstacle.width)];
        else
            f_x_TV = [f_x_TV;0 0 0 1 0]; % e_y
            v_x_TV = [v_x_TV;min(obstacle.width)];
        end
        TV_x{obs}   = obstacle.duration(1)/T_sampling:obstacle.duration(2)/T_sampling;
    end

end