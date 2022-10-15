function plotObstacles(Obstacles)
    for obstacle = Obstacles
        rectangle('Position', [min(obstacle.duration) min(obstacle.width) abs(obstacle.duration(2)-obstacle.duration(1)) abs(obstacle.width(2)-obstacle.width(1))], 'Curvature', 0.3)
    end
end