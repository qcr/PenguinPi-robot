classdef PiBot < handle

    properties(Access = public)
        TCP_MOTORS;
        TCP_CAMERA;
        TCP_TAGS;
    end

    properties (Access = private, Constant)
        TIMEOUT = 10;

        PORT_MOTORS = 43900; % some random ports that should be unused as they are above 2000?
        PORT_CAMERAS = 43901;
        PORT_TAGS = 43902;

        IMAGE_WIDTH = 640/2;
        IMAGE_HEIGHT = 480/2;
        IMAGE_SIZE = PiBot.IMAGE_WIDTH * PiBot.IMAGE_HEIGHT * 3;

        FN_ARG_SEPARATOR = ','
        FN_GET_IMAGE = 'getImageFromCamera'
        FN_MOTOR_SPEEDS = 'setMotorSpeeds'
        FN_MOTOR_SPEEDS_PROFILE = 'setMotorSpeedsProfile'
        FN_MOTOR_TICKS = 'getMotorTicks'
        FN_APRIL_TAGS = 'getTags'
        FN_MOTOR_ENCODERS = 'getMotorEncoders'
        FN_DISPLAY_VALUE = 'setDisplayValue'
        FN_DISPLAY_MODE = 'setDisplayMode'
        FN_ALL_STOP = 'stopAll'
        FN_MOTOR_MOVE = 'setMotorMove'
    end

    methods
        function obj = PiBot(address)
            %PiBot.PiBot Construct a PiBot object
            %
            % PB = PiBot(IP) creates an object used for communications with the robot
            % connected via the IP address which is given as a string in dot format.
            %
            % See also PiBot.setMotorSpeeds, PiBot.getMotorTicks,
            % PiBot.setDisplayValue, PiBot.setDisplayMode.

            obj.TCP_MOTORS = tcpip(address, PiBot.PORT_MOTORS, 'NetworkRole', 'client', 'Timeout', PiBot.TIMEOUT);
            obj.TCP_CAMERA = tcpip(address, PiBot.PORT_CAMERAS, 'NetworkRole', 'client', 'Timeout', PiBot.TIMEOUT);
            obj.TCP_TAGS = tcpip(address, PiBot.PORT_TAGS, 'NetworkRole', 'client', 'Timeout', PiBot.TIMEOUT);

            % Configure the TCPIP objects
            %obj.TCP_CAMERA.Timeout = PiBot.TIMEOUT;
            %obj.TCP_MOTORS.Timeout = PiBot.TIMEOUT;
            obj.TCP_CAMERA.InputBufferSize = PiBot.IMAGE_SIZE;

        end

        function delete(obj)
            delete(obj.TCP_MOTORS);
            delete(obj.TCP_CAMERA);
        end

        function imgVect = getVectorFromCamera(obj)
            fopen(obj.TCP_CAMERA);
            fprintf(obj.TCP_CAMERA, [PiBot.FN_GET_IMAGE PiBot.FN_ARG_SEPARATOR '100']);
            imgVect = fread(obj.TCP_CAMERA, PiBot.IMAGE_SIZE, 'uint8')./255;
            fclose(obj.TCP_CAMERA);
        end

        function img = getImageFromCamera(obj)
            img = [];

            % Attempt to retrieve the image
            try
                vector = obj.getVectorFromCamera();
            catch error
                warning('Empty image array returned from RPi');
                return;
            end

            % Convert the image to MATLAB format (if it's the correct size)
            assert(length(vector) == PiBot.IMAGE_SIZE, 'Size of data received (%d) did not match expected image size (%d). Empty image array returned!\n', length(vector), PiBot.IMAGE_SIZE);
            img = reshape([vector(1:3:end); vector(2:3:end); vector(3:3:end)],PiBot.IMAGE_WIDTH, PiBot.IMAGE_HEIGHT, 3);
        end

        function setMotorSpeeds(obj, varargin)
            %PiBot.setMotorSpeeds  Set the speeds of the motors
            %
            % PB.setMotorSpeeds(SA, SB) sets the speeds of the two motors to the values
            % SA and SB.
            %
            % PB.setMotorSpeeds(SPEED, T) sets the speeds of the two motors to the values
            % in the 2-vector SPEED = [SA SB] and the motion runs for T seconds. 
            % Timing is done locally on the RPi.
            %            %
            % PB.setMotorSpeeds(SPEED, T, ACC) as above but the speed ramps up and down at the end
            % of the motion over ACC seconds.  The constant velocity part of the motion lasts for
            % T-ACC seconds, the total motion time is T+2*ACC seconds.  This profile moves the same 
            % distance as a rectangular speed profile over T seconds.
            %
            % Note::
            % - This method sets the motor voltage which is somewhat correlated to
            %   rotational speed.
            %
            % See also PiBot.stop.

            if length(varargin{1}) == 1
                % then (SA, SB) classic format
                speed(1) = varargin{1}; speed(2) = varargin{2};
                
                assert(all(isreal(speed)), 'arguments must be real');
                assert(all(fix(speed)==speed), 'arguments must have an integer value');
                assert(all(speed>=-100 & speed<=100), 'arguments must be in the range -100 to 100');
                
                data = [PiBot.FN_MOTOR_SPEEDS];
                data = [data PiBot.FN_ARG_SEPARATOR num2str(speed(1)) ...
                    PiBot.FN_ARG_SEPARATOR num2str(speed(2))];
                
                fopen(obj.TCP_MOTORS);
                fprintf(obj.TCP_MOTORS, data);
                fclose(obj.TCP_MOTORS);
                
            elseif length(varargin{1}) == 2
                % then (SPEED), (SPEED, T) or (SPEED, T, A)
                
                speed = varargin{1};
                assert(all(isreal(speed)), 'arguments must be real');
                assert(all(fix(speed)==speed), 'arguments must have an integer value');
                assert(all(speed>=-100 & speed<=100), 'arguments must be in the range -100 to 100');
                
                assert(nargin >= 3, 'must specify duration');
                duration = varargin{2};
                
                assert(duration > 0, 'duration must be positive');
                assert(duration < PiBot.TIMEOUT*0.8, 'duration must be < network timeout');

                if nargin >= 4
                    accel = varargin{3};
                else
                    accel = 0;
                end
                assert(accel < duration/2, 'accel must be < accel/2');
                
                data = [PiBot.FN_MOTOR_SPEEDS_PROFILE];
                data = [data PiBot.FN_ARG_SEPARATOR num2str(speed(1)) ...
                        PiBot.FN_ARG_SEPARATOR num2str(speed(2)) ...
                        PiBot.FN_ARG_SEPARATOR num2str(duration) ...
                        PiBot.FN_ARG_SEPARATOR num2str(accel) ...
                        ];
                    
                    fopen(obj.TCP_MOTORS);
                    fprintf(obj.TCP_MOTORS, data);
                    
                    s = fgetl(obj.TCP_MOTORS); % wait for completion
                    fclose(obj.TCP_MOTORS);
            end
        end

        
%         function setMotorMove(obj, varargin)
%             %PiBot.setMotorMove  Set the movement of the motors
%             %
%             % PB.setMotorSpeeds(MA, MB) sets the angular change of the two motors to the values
%             % MA and MB.
%             %
%             % PB.setMotorSpeeds(MOVE) sets the speeds of the two motors to the values
%             % in the 2-vector MOVE = [MA MB].
%             %
%             % See also setMotorSpeed.
%             
%             if nargin == 2
%                 motors = varargin{1};
%             elseif nargin == 3
%                 motors = [varargin{1} varargin{2}];
%             else
%                 error('incorrect number of arguments provided');
%             end
%                 
%             assert(all(isreal(motors)), 'arguments must be real');
%             assert(all(fix(motors)==motors), 'arguments must have an integer value');
%             
%             data = [PiBot.FN_MOTOR_MOVE];
%             data = [data PiBot.FN_ARG_SEPARATOR num2str(motors(1)) PiBot.FN_ARG_SEPARATOR num2str(motors(2))];
%              
%             fopen(obj.TCP_MOTORS);
%             fprintf(obj.TCP_MOTORS, data);
%             fclose(obj.TCP_MOTORS);
%         end
        

        function stop(obj)
            %PiBot.stop  Stop all motors
            %
            % PB.stop() stops all motors.
            %
            % See also PiBot.setMotorSpeed.

            obj.setMotorSpeeds(0, 0);
        end

        function reset(obj)
            %PiBot.reset  Stop all motors and reset encoders
            %
            % PB.reset() stop all motors and reset encoders.
            %
            % See also PiBot.stop, PiBot.setMotorSpeed.

            data = [PiBot.FN_ALL_STOP];

            fopen(obj.TCP_MOTORS);
            fprintf(obj.TCP_MOTORS, data);
            fclose(obj.TCP_MOTORS);
        end

        function ticks = getMotorTicks(obj)
        %PiBot.getMotorTicks   Get motor angles in degrees
        %
        % PB.getMotorTicks() returns a 2-vector containing the current shaft angle
        % of the two robot motors.
        %
        % Note::
        % - The returned values have been rescaled to units of degrees.
        % - The encoder counter on the robot has a 16-bit signed value.

            data = [PiBot.FN_MOTOR_TICKS,PiBot.FN_ARG_SEPARATOR 'A']; % needed for the Pi code
            fopen(obj.TCP_MOTORS);
            fprintf(obj.TCP_MOTORS, data);

            s = fgetl(obj.TCP_MOTORS);
            fclose(obj.TCP_MOTORS);

            % Convert ticks to numerical array
            ticks = sscanf(s,'%d');
        end

        function tags = getTags(obj)
            elements = 5;  %elements per tag.
            data = [PiBot.FN_APRIL_TAGS,PiBot.FN_ARG_SEPARATOR 'A']; % needed for the Pi code
            fopen(obj.TCP_TAGS);
            fprintf(obj.TCP_TAGS, data);

            s = fgetl(obj.TCP_TAGS);
            fclose(obj.TCP_TAGS);

            % Convert ticks to numerical array
            tempTags = strsplit(s,{';', ' '});
            tempTags(end) = [];
            
            numTags = size(tempTags,2)/elements;

            tags = zeros(numTags,elements);

            for i = 1:numTags
                for ii = 1:elements
                   tags(i,ii) = cell2mat(tempTags(((i-1)*5)+ii));
                end
            end
        end

        function ticks = getMotorEncoders(obj)
        %PiBot.getMotorEncoders   Get motor encoder values
        %
        % PB.getMotorTicks() returns a 2-vector containing the current encoder
        % values of the two robot motors.
        %
        % Note::
        % - The encoder counter on the robot has a 16-bit signed value.

            data = [PiBot.FN_MOTOR_ENCODERS,PiBot.FN_ARG_SEPARATOR 'A']; % needed for the Pi code
            fopen(obj.TCP_MOTORS);
            fprintf(obj.TCP_MOTORS, data);

            s = fgetl(obj.TCP_MOTORS);
            fclose(obj.TCP_MOTORS);

            % Convert ticks to numerical array
            ticks = sscanf(s,'%d');
        end

        function setDisplayValue(obj, val)
        %PiBot.setDisplayValue  Write to the robot display
        %
        % PB.setDisplayValue(V) writes the value of V to the robot's display, using
        % the current mode.  The range of allowable values depends on the mode:
        %  - hexadecimal 0 to 255
        %  - unsigned decimal 0 to 99
        %  - signed decimal -9 to 9
        %
        % See also PiBot.setDisplayMode.

            assert(isreal(val), 'argument must be real');
            assert(fix(val)==val, 'argument must have an integer value');

            data = [PiBot.FN_DISPLAY_VALUE];
            data = [data PiBot.FN_ARG_SEPARATOR num2str(val)];

            fopen(obj.TCP_MOTORS);
            fprintf(obj.TCP_MOTORS, data);
            fclose(obj.TCP_MOTORS);
        end

        function setDisplayMode(obj, val)
        %PiBot.setDisplayMode  Set the robot display mode
        %
        % PB.setDisplayMode(M) sets the numerical mode for the robot's display:
        %  - 'x' hexadecimal
        %  - 'u' unsigned decimal
        %  - 'd' signed decimal -9 to 9
        %
        % In decimal modes the decimal point on the right-hand digit is lit.
        %
        % See also PiBot.setDisplayValue.
            data = [PiBot.FN_DISPLAY_MODE];
            data = [data PiBot.FN_ARG_SEPARATOR val];

            fopen(obj.TCP_MOTORS);
            fprintf(obj.TCP_MOTORS, data);
            fclose(obj.TCP_MOTORS);
        end

    end
end
