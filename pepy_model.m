% To implement the Pepy Bicycle Kinematic Model as a class

classdef pepy_model
    properties
        x ; % X position
        y ;% Y position
        psi;% yaw

        delta; % steering angle
        vel; %velocity

        l_r % distance between C and rear axle
        l_f % distance between C and front axle
        L % wheelbase length

        max_delta; % maximum steering angle
    end
    methods
        function obj = pepy_model(l_f, l_r, max_steer, v)
            % constructor
            obj.l_f = l_f;
            obj.l_r = l_r;
            obj.L = obj.l_f + obj.l_r;
            obj.max_delta = max_steer;
            obj.vel = v;
            obj.x = 0;
            obj.y = 0;
            obj.psi = 0;
            obj.delta = 0;

        end
        function obj = reset(obj)
            obj.x = 0;
            obj.y = 0;
            obj.psi = 0;
            obj.delta = 0;
        end
        function obj = step(obj, t, delta)
            obj.delta = max(-obj.max_delta, min(obj.max_delta, delta)); % set new steering angle

            % update velocities
            vel_yaw = (obj.vel/obj.L)*tan(obj.delta);
            vel_x = obj.vel*cos(obj.psi);
            vel_y = obj.vel*sin(obj.psi);

            % update state
            obj.x = obj.x + vel_x * t;
            obj.y = obj.y + vel_y * t;

            obj.psi = obj.psi + vel_yaw * t;
            
            % Normalize yaw to [-pi, pi]
            obj.psi = mod(obj.psi + pi, 2*pi) - pi;
            
        end
        function [x, y, psi] = get_state(obj)
            % return the current state
            x = obj.x;
            y = obj.y;
            psi = obj.psi;
        end
    end
end

