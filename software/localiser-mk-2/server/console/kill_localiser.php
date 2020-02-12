<!-- TODO: get PID from file -->
<?php

    if(isset($_POST)) {
        deliver_response(200, "success", $_POST);
    }
    
    function deliver_response($status, $status_message, $data) {
        $string = file_get_contents("localiser_PID.txt");

        // TODO GET ONLY NUMBER 
        posix_kill($string,9);
        $success=false;
        if(posix_get_last_error()==1) /* EPERM */
            $success=true;
        header("HTTP/1.1 $status $status_message");
        $response['status'] = $status;
        $response['status_message'] = $status_message;
        $response['data'] = "Killed PID " . $string . " with code " . $success;
    
    $json_response = json_encode($response);
        echo $json_response;
    }
    
?>