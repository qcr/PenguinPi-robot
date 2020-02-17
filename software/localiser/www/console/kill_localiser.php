<!-- TODO: get PID from file -->
<?php

    if(isset($_POST)) {
        deliver_response(200, "success", $_POST);
    }
    
    function deliver_response($status, $status_message, $data) {
        
        $shell_user = exec("whoami"); 
        $string = file_get_contents("localiser_PID.txt");
        $PID = trim($string);
        $signal = 9;

        $success=false;
        posix_kill($PID,SIGKILL);
        #exec("kill -9 $PID");
        if(posix_get_last_error()==1) /* EPERM */
            $success=true;
        header("HTTP/1.1 $status $status_message");
        $response['status'] = $status;
        $response['status_message'] = $status_message;
        $response['data'] = "User " . $shell_user . " sent signal " . $signal . " to PID " . $PID . " with code " . $success;
    
    $json_response = json_encode($response);
        echo $json_response;
    }
    
?>