classdef tank
    %UNTITLED Summary of this class goes here
    %   Tank object

    properties
        BodyTree
    end

    methods
        function obj = tank(sensorArragement)
            %UNTITLED Construct an instance of this class
            %   Create a new tank object. sensorArragement is either 1
            %   (360) or 4 (corners). 
            if sensorArragement ~= 1 || sensorArragement ~= 4
                error("SensorArragement must be 1 or 4")
            end

            
            
        end

        function outputArg = method1(obj,inputArg)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            outputArg = obj.Property1 + inputArg;
        end
    end
end