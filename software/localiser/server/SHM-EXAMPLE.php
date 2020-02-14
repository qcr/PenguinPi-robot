
<?php
   
   $home = getenv ("HOME");

   $shm_key =  ftok( '/etc', 'x' );

    // Get a semaphore to ensure exclusive access to the shared memory

//----- SHARED MEMORY CONFIGURATION -----
$SHARED_MEMORY_KEY = 672213396;   	//Shared memory unique key (SAME AS RPi App  KEY)

//Create the semaphore
do {
    $semaphore_id = sem_get($SEMAPHORE_KEY, 1);		//Creates, or gets if already present, a semaphore
} while ($semaphore_id === false);

//Acquire the semaphore
if (!sem_acquire($semaphore_id))						//If not available this will stall until the semaphore is released by the other process
{
    echo "Failed to acquire semaphore $semaphore_id<br />";
    sem_remove($semaphore_id);						//Use even if we didn't create the semaphore as something has gone wrong and its usually debugging so lets no lock up this semaphore key
    exit;
}

   $shm_id = shmop_open( $shm_key, "a", 0644, 24);
   if (!$shm_id) {
       echo "Couldn't create shared memory segment\n";
   }
   
   // Get shared memory block's size
   $shm_size = shmop_size($shm_id);
   echo "SHM block with key " . $shm_key . " and SHM Block Size: " . $shm_size . " has been opened.\n";
   
//    // Lets write a test string into shared memory
//    $shm_bytes_written = shmop_write($shm_id, "my shared memory block", 0);
//    if ($shm_bytes_written != strlen("my shared memory block")) {
//        echo "Couldn't write the entire length of data\n";
//    }
   
   // Now lets read the string back
   $pose_string = shmop_read($shm_id, 0, $shm_size);
   if (!$pose_string) {
       echo "Couldn't read from shared memory block\n";
   }

   // Unpack with machine-depended double representation
//    $pose_array = array_slice(unpack('dx/dy/dtheta', $pose_string), 0);

    $pose_array = unpack('d3val', $pose_string);
     echo "Shared memory bytes: ";

     $x= $pose_array['val1'];
     $y= $pose_array['val2'];
     $theta = $pose_array['val3'];

     echo "Pose:" . $x . "," . $y . "," . $theta . "\n";
   
//    //Now lets delete the block and close the shared memory segment
//    if (!shmop_delete($shm_id)) {
//        echo "Couldn't mark shared memory block for deletion.";
//    }
//    shmop_close($shm_id);
      //Release the semaphore
if (!sem_release($semaphore_id))				//Must be called after sem_acquire() so that another process can acquire the semaphore
echo "Failed to release $semaphore_id semaphore<br />";

   ?>
   