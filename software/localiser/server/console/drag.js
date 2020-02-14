// Make the DIV element draggable:


function dragElement(elmnt) {

    // Attach event listeners
    var pos1 = 0, pos2 = 0, pos3 = 0, pos4 = 0;
      // if present, the header is where you move the DIV from:
    document.getElementById(elmnt.id).onmousedown = dragMouseDown;

    function dragMouseDown(e) {
      console.log("drag mouse down")
      e = e || window.event;
      e.preventDefault();
      // get the mouse cursor position at startup:
      pos3 = e.clientX;
      pos4 = e.clientY;
      document.onmouseup = closeDragElement;
      // call a function whenever the cursor moves:
      document.onmousemove = elementDrag;
    }
  
    function elementDrag(e) {
      console.log("element drag")
      e = e || window.event;
      e.preventDefault();
      // calculate the new cursor position:
      pos1 = pos3 - e.clientX;
      pos2 = pos4 - e.clientY;
      pos3 = e.clientX;
      pos4 = e.clientY;
      // set the element's new position:
      elmnt.style.top = (elmnt.offsetTop - pos2) + "px";
      elmnt.style.left = (elmnt.offsetLeft - pos1) + "px";
    }
  
    function closeDragElement() {
      // stop moving when mouse button is released:
      document.onmouseup = null;
      document.onmousemove = null;
    }
  }

  $(document).ready( function() {


    $.ajax({
      type: "POST",
      url: "kill_localiser.php" ,
      data: {},
      success : function() { 

          // here is the code that will run on client side after running clear.php on server

          // function below reloads current page
          console.log("sent request to kill localiser script");

      }
  });

    // Attach event listeners to draggable markers 

    var tie_points = ["NW","NE","SE","SW"];

    for (var i=0; i< tie_points.length; i++){
        dragElement(document.getElementById(tie_points[i]));
    }
  });

