%PiBot MATLAB interface for PenguinPi robot
%
% A PiBot object provides methods to obtain status from, and control, a wireless
% PenguinPi robot.
%
% Methods::
% 
% PiBot         constructor
% char          info in human readable form
% display       display information about object
%
% setVelocity   set motor velocity
% stop          stop all motors
%
% getImage      get image from camera
%
% setLed        set LEDS 2-4
% pulseLed      pulse LEDS 2-4
% setLedArray   set the blue LED array
% getButton     get user button status
% getDIP        get value of DIP switch
%
% getCurrent    get battery voltage
% getVoltage    get battery current
%
% (c) 2019 Peter Corke

    
classdef PiBot < handle

    
    properties(Access = public)
        url
    end
    
    methods
        function obj = PiBot(address)
            %PiBot.PiBot Construct a PiBot object
            %
            % PB = PiBot(IP) creates an object used for communications with the robot
            % connected via the IP address which is given as a string in dot format.
            %
            % Notes::
            % - This is a handle class object.
            %
            % See also PiBot.setVelocity, PiBot.getImage.
            
            obj.url = string(address) + ":8080";
        end
        
        function s = char(obj)
        % Link.cchar Convert to string
        %
        % s = PB.char() is a string showing robot parameters in a compact single line format.
        %  
        %   See also PiBot.display.
  
            s = sprintf('PenguinPi robot at %s', obj.url');
        end
        
        function display(obj)
        % PibBot.display Display parameters
        %  
        %  PB.display() displays the robot parameters in compact single line format.  
        %
        % See also PiBot.char.
            disp( char(obj));
        end
        
        function delete(obj)
        end
        
        function stat = setVelocity(obj, varargin)
            %PiBot.setVelocity  Set the speeds of the motors
            %
            % PB.setVelocity(Vleft, Vright) sets the velocities of the two motors to the values
            % Vleft and Vright respectively.
            %
            % PB.setVelocity(VEL, T) sets the speeds of the two motors to the values
            % in the 2-vector VEL = [Vleft, Vright] and the motion runs for T seconds.
            % Timing is done locally on the RPi.
            %
            % PB.setVelocity(VEL, T, ACC) as above but the speed ramps up and down at the end
            % of the motion over ACC seconds.  The constant velocity part of the motion lasts for
            % T-ACC seconds, the total motion time is T+2*ACC seconds.  This profile moves the same
            % distance as a rectangular speed profile over T seconds.
            %
            % STAT = PB.setVelocity(...) as for any of the above call formats, but returns a structure
            % that contains the encoder count and dead-reckoned pose at the end of the motion.
            %
            % See also PiBot.stop.
            
            if length(varargin{1}) == 1
                % then (SA, SB) classic format
                vel(1) = varargin{1}; vel(2) = varargin{2};
                
                assert(all(isreal(vel)), 'arguments must be real');
                vel = round(vel);  % convert to int
                vel = min(max(vel, -100), 100);  % clip
                
                json = webread(obj.url+"/robot/set/velocity", "value", vel);
                
                
            elseif length(varargin{1}) == 2
                % then (SPEED), (SPEED, T) or (SPEED, T, A)
                
                vel = varargin{1};
                assert(all(isreal(vel)), 'arguments must be real');
                vel = round(vel);  % convert to int
                vel = min(max(vel, -100), 100);  % clip
                
                if length(varargin) == 1
                    json = webread(obj.url+"/robot/set/velocity", "value", vel);
                elseif length(varargin) >= 2
                    duration = varargin{2};
                    assert(duration > 0, 'duration must be positive');
                    assert(duration < 20, 'duration must be < network timeout');
                    
                    
                    if length(varargin) >= 3
                        accel = varargin{3};
                        assert(accel < duration/2, 'accel must be < accel/2');
                        json = webread(obj.url+"/robot/set/velocity", "value", vel, "time", duration, "acceleration", accel);
                    else
                        json = webread(obj.url+"/robot/set/velocity", "value", vel, "time", duration);
                    end
                end
            end
            
            if nargout > 0
                stat = jsondecode(json);
            end
        end
        
        function stat = stop(obj)
            %PiBot.stop  Stop all motors
            %
            % PB.stop() stops all motors.
            %
            % STAT = PB.stop() as above but returns a structure that contains the encoder 
            % count and dead-reckoned pose at the end of the motion.
            %
            % See also PiBot.setVelocity.
            
            json = webread(obj.url+"/robot/stop");
            if nargout > 0
                stat = jsondecode(json);
            end
        end
        
        function reset(obj)
            %PiBot.reset  Stop all motors and reset encoders
            %
            % PB.reset() stop all motors and reset encoders.
            %
            % See also PiBot.stop, PiBot.setMotorSpeed.
            
        end
        
        function v = getVoltage(obj)
            %PiBot.getVoltage  Get battery voltage
            %
            % PB.getVoltage() is the battery voltage in volts.
            %
            % See also PiBot.getCurrent.
            v = str2num( webread(obj.url+"/battery/get/voltage") ) /1000.0;
        end
        
        function c = getCurrent(obj)
            %PiBot.getCurrent  Get battery current
            %
            % PB.getCurrent() is the battery voltage in amps.
            %
            % See also PiBot.getCurrent.
            c = str2num( webread(obj.url+"/battery/get/current") ) /1000.0;
        end
        
        function setLed(obj, i, s)
            %PiBot.setLed  Set yellow LED
            %
            % PB.setLed(num, state) sets the yellow LED num to the state.
            %
            % Notes::
            % - LED number must in the range 2 to 4.
            % - state is an integer (0 or 1), or a logical (false or true).
            %
            % See also PiBot.pulseLed.
            assert(i>=2 && i<=4, 'invalid LED');
            assert(s==0 || s==1, 'invalid state')
            
            webread(obj.url+"/led/set/state", "id", i, "value", s);
        end
        
        function pulseLed(obj, i, duration)
            %PiBot.pulseLed  Pulse yellow LED
            %
            % PB.pulseLed(num, time) pulses the yellow LED num to the on state
            % for time (in seconds).
            %
            % Notes::
            % - LED number must in the range 2 to 4.
            % - pulse time is in the range 0 to 0.255 seconds.
            %
            % See also PiBot.pulseLed.
            assert(i>=2 && i<=4, 'invalid LED');
            assert(duration>0 && duration<=0.255, 'invalid duration')
            
            webread(obj.url+"/led/set/count", "id", i, "value", duration*1000);
        end
        
        function d = getDIP(obj)
            %PiBot.getDIP  Get value of DIP switch
            %
            % PB.getDIP() is the value of the DIP switch. 
            %
            % Notes::
            % - SW4 is the LSB
            % - SW3 and SW4 are used by the onboard software
            d = webread(obj.url+"/hat/dip/get");
        end
        
        function b = getButton(obj)
            %PiBot.getButton  Get user button
            %
            % PB.getButton() is the number of times the rightmost button has been
            % pushed since the last call to this function.

            b = webread(obj.url+"/hat/button/get");
        end
        
        
        function setLedArray(obj, v)
            %PiBot.setLedArray sets the blue LED array
            %
            % PB.setLedArray(v) sets the blue LED array on top of the robot to the
            % 16-bit integer value v.  Each bit represents an LED in the array.  The
            % LSB is the bottom right LED and number increasing upwards and to the
            % left.
            
            webread(obj.url+"/hat/ledarray/set", "value", v);
        end
        
        
        function img = getImage(obj)
            %PiBot.getImagee  Get image from camera
            %
            % PB.getImage() is an RGB image captured from the front camera of the
            % robot.
            %
            % See also PiBot.getCurrent.
            img = webread(obj.url+"/camera/get");
        end
    end
end

    function print_json(j1, j2)
        state1 = jsondecode(j1);
        state2 = jsondecode(j2);
        fprintf('Initial\n');
        state1.encoder
        state1.pose
        fprintf('Final\n');
        state2.encoder
        state2.pose
    end