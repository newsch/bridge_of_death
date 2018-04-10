function runCourse(Times,Vl,Vr)
%RUNCOURSE  Run the robot through a precalculated course
%   Times is an array of pause lengths between commands.
%   Vl is an array of speeds for the left wheel.
%   Vr is an array of speeds for the right wheel.
    
    cleanupObj = onCleanup(@cleanMeUp);

    sub = rossubscriber('/bump');
    pub = rospublisher('/raw_vel');
    message = rosmessage(pub);

    disp("Ready to run course.")
    disp("Press any key to start.")
    disp("Press CTRL+C to stop.")
    pause();  % wait for input
    disp("Starting run.")
    
    % loop through times and velocities
    for i = 1:length(Times)
        message.Data = [Vl(i), Vr(i)];
        send(pub, message);
        pause(Times(i));
    end

    function cleanMeUp()
    % Called when the function is aborted or finished.
        disp("Stopping robut.")
        message.Data = [0,0];
        send(pub,message);
    end
end

