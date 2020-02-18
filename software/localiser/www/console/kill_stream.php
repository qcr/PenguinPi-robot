<!-- TODO: get PID from file -->
<?php

    if(isset($_POST)) {
        deliver_response(200, "success", $_POST);
    }
    
    function deliver_response($status, $status_message, $data) {

        $new_speed = $_POST['new_speed'];

        // Write new speed to file
        $shell_user = exec("whoami"); 
        $string = file_get_contents("/var/www/penguinpi/videostream.pid");
        $PID = trim($string);
        $kill_command = "kill " . $PID;
        $output = shell_exec($kill_command);
        $myfile = fopen("/var/www/penguinpi/shutterspeed", "w") or die("Unable to open file!");
        fwrite($myfile, $new_speed);
        fclose($myfile);
        header("HTTP/1.1 $status $status_message");
        $response['status'] = $status;
        $response['status_message'] = $status_message;
        echo "User " . $shell_user . " sent signal " . $signal . " to PID " . $PID . " with code " . $output . " and requested new speed " . $new_speed;
    }
?>