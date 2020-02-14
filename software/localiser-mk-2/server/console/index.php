<?php 

    $string = file_get_contents("tie_points.json");
    $tie_points = json_decode($string, true); 

    $tie_point_html = "";

    foreach($tie_points as $key => $value){
        $tie_point_html .= "<div class=\"tie_point\" id=\"" . $key ."\" style=\"top:". $value['y'] . "px; left: " . $value['x'] . "px;\"><img src=\"icon.png\"></div>";
    }

?>

<!doctype html>

<html>
    <head>
        <script type="text/javascript" src="jquery-3.4.1.min.js"></script>
        <script type="text/javascript" src="drag.js"></script>
        <link rel="stylesheet" type="text/css" href="style.css">
    </head>

    <body>

    <div id="arena_image"> 
        <img src="/camera/get/arena.jpg">
        <div> 
            <?php echo $tie_point_html; ?>
        </div>
    </div>



    </body>
</html>


