%   This class defines a template for creating a custom sensor that can be
%   used by the robotSensor in Robot Scenario. This sensor adaptor allows your
%   custom sensor model to be called during a Robot Scenario simulation.
%
%   To access documentation for how to define a sensor adaptor, enter the
%   following in the MATLAB command prompt:
%
%    >> doc robotics.SensorAdaptor
%
%   For the implementation of the same interface, see the following
%   robotics.SensorAdaptor class:
%
%    >> edit robotics.internal.scenario.sensor.GPSAdaptor
%
%
%   To use this custom sensor model for scenario simulation, follow the
%   steps outlined below and complete the class definition. Then, save this
%   file somewhere on the MATLAB path. You can add a folder to the path
%   using the ADDPATH function.

classdef CustomUltrasonicSensor < robotics.SensorAdaptor

    %---------------------------------------------------------------------
    % Define update rate of your custom sensor. Use nan if you want
    % your sensor to update as frequently as the scenario.
    properties
        UpdateRate = nan
    end

    methods
        function obj = CustomUltrasonicSensor(sensorModel)
            %MyCustomSensor

            % Invoke the base class constructor to wrap around the sensor
            % model.
            obj@robotics.SensorAdaptor(sensorModel);
            obj.UpdateRate = obj.SensorModel.UpdateRate;
        end


        function setup(obj, scene, platform) %#ok<INUSD>
            %setup

            %--------------------------------------------------------------
            % Place your code here to setup your sensor model based on
            % scenario and platform information. This method is called when
            % creating a robotSensor object that contains this custom
            % sensor adaptor.
            %--------------------------------------------------------------
        end

        function [det, isValid] = read(obj, scene, platform, sensor, t)
            %read

            actor = [];
            for idx = 1:numel(scene.Platforms)
                if ~strcmp(scene.Platforms(idx).Name, sensor.MountingBodyName)
                    %trans = scene.TransformTree.getTransform(scene.Platforms(idx).Name,"ENU");
                    trans = scene.TransformTree.getTransform(sensor.Name, scene.Platforms(idx).Name, t);
                    eul = rotm2eul(trans(1:3,1:3),'ZYX');
                    act = struct("ActorID",idx,"Position",trans(1:3,4)',...
                           "Speed",0,"Yaw", eul(1), "Pitch", eul(2));
                    actor = [ actor, act ];

                end
            end

            [det, isValid] = obj.SensorModel(actor, t);
        end

        function reset(obj)
            %reset

            obj.SensorModel.reset();
            obj.SensorModel.release();
        end

        function out = getEmptyOutputs(obj) %#ok<MANU>
            %getEmptyOutputs

            out = {[], false};
        end

        function out = get.UpdateRate(obj)
            out = obj.SensorModel.UpdateRate;
        end

        function set.UpdateRate(obj, r)
            obj.SensorModel.UpdateRate = r;
        end
    end
end