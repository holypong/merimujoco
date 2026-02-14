import numpy as np
import time
import argparse
import sys
import logging
from redis_transfer import RedisTransfer

# Logger configuration (INFO level, console output only)
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s [%(levelname)s] %(message)s',
    datefmt='%H:%M:%S',
    handlers=[logging.StreamHandler()]
)
logger = logging.getLogger(__name__)

# 20260208 calc_dance_motion.py created
# Send full-body dance motion with sine curve to Redis

REDIS_HOST = 'localhost'
REDIS_PORT = 6379
REDIS_KEY = 'meridis_calc_pub'  # Key to write calculation results

class DanceMotion:
    def __init__(self, host=REDIS_HOST, port=REDIS_PORT, redis_key=REDIS_KEY):
        """Dance motion generator class
        
        Args:
            host (str): Redis server hostname
            port (int): Redis server port number
            redis_key (str): Redis key to write data
        """
        self.redis_transfer = RedisTransfer(host=host, port=port, redis_key=redis_key)
        self.x = 0.0  # Variable for sine curve
        self.meridis_motion = [0.0] * 90  # 90-element motion data
        
    def calculate_motion(self):
        """Calculate dance motion with sine curve for full-body movement"""
        # Initialize
        self.meridis_motion = [0.0] * 90

        def r(value):
            return round(float(value), 2)
        
        # Dance motion with sine curve for full-body wiggling
        self.meridis_motion[21] = r(-np.sin(self.x) * 30)               # Head yaw
        self.meridis_motion[51] = r(np.sin(self.x) * 20)                # Waist yaw

        self.meridis_motion[23] = r(np.sin(self.x) * 20)                # Left shoulder pitch
        self.meridis_motion[25] = r(-np.sin(self.x * 2) * 10 + 30)      # Left shoulder roll
        self.meridis_motion[27] = r(np.sin(self.x) * 10 + 10)           # Left elbow yaw
        self.meridis_motion[29] = r(np.sin(self.x) * 45 - 45)           # Left elbow pitch
        self.meridis_motion[31] = r(np.sin(self.x) * 5)                 # Left hip yaw

        self.meridis_motion[53] = r(-np.sin(self.x) * 20)               # Right shoulder pitch
        self.meridis_motion[55] = r(-np.sin(self.x * 2) * 10 + 30)      # Right shoulder roll
        self.meridis_motion[57] = r(-np.sin(self.x) * 10 + 10)          # Right elbow yaw
        self.meridis_motion[59] = r(-np.sin(self.x) * 45 - 45)          # Right elbow pitch
        self.meridis_motion[61] = r(-np.sin(self.x) * 5)                # Right hip yaw

        self.meridis_motion[33] = r(-np.sin(self.x) * 5)                # Left hip roll
        self.meridis_motion[35] = r(np.sin(self.x * 2) * 5 - 5)         # Left hip pitch
        self.meridis_motion[37] = r(-np.sin(self.x * 2) * 10 + 10)      # Left knee pitch
        self.meridis_motion[39] = r(np.sin(self.x * 2) * 5 - 5)         # Left ankle pitch
        self.meridis_motion[41] = r(np.sin(self.x) * 5)                 # Left ankle roll

        self.meridis_motion[63] = r(np.sin(self.x) * 5)                 # Right hip roll
        self.meridis_motion[65] = r(-np.sin(self.x * 2) * 5 - 5)        # Right hip pitch
        self.meridis_motion[67] = r(np.sin(self.x * 2) * 10 + 10)       # Right knee pitch
        self.meridis_motion[69] = r(-np.sin(self.x * 2) * 5 - 5)        # Right ankle pitch
        self.meridis_motion[71] = r(-np.sin(self.x) * 5)                # Right ankle roll
        
        return self.meridis_motion
    
    def send_motion(self):
        """Send calculated motion data to Redis"""
        if not self.redis_transfer.is_connected:
            logger.error("Not connected to Redis")
            return False
        
        motion_data = self.calculate_motion()
        self.redis_transfer.set_data(data=motion_data)
        return True
    
    def update_x(self, increment=0.05):
        """Update x variable for sine curve
        
        Args:
            increment (float): Increment value for x
        """
        self.x += increment
        if self.x > 2 * np.pi:
            self.x -= 2 * np.pi
    
    def run(self, duration=None, frequency=50):
        """Continuously send dance motion
        
        Args:
            duration (float, optional): Execution duration in seconds. None for infinite loop
            frequency (int): Transmission frequency in Hz
        """
        interval = 1.0 / frequency
        start_time = time.time()
        loop_count = 0
        
        logger.info(f"Dance motion transmission started (Key: {self.redis_transfer.redis_key}, Frequency: {frequency}Hz)")
        if duration:
            logger.info(f"Duration: {duration}s")
        else:
            logger.info("Duration: Unlimited (Press Ctrl+C to stop)")
        
        try:
            while True:
                loop_start = time.time()
                
                # Motion calculation and transmission
                if self.send_motion():
                    loop_count += 1
                    if loop_count % (frequency * 5) == 0:  # Progress display every 5 seconds
                        elapsed = time.time() - start_time
                        logger.info(f"Elapsed: {elapsed:.1f}s, Loops: {loop_count}")
                
                # Update x
                self.update_x(increment=0.05)
                
                # Check termination
                if duration and (time.time() - start_time) >= duration:
                    logger.info(f"Specified duration ({duration}s) elapsed")
                    break
                
                # Wait until next loop
                elapsed = time.time() - loop_start
                sleep_time = max(0, interval - elapsed)
                if sleep_time > 0:
                    time.sleep(sleep_time)
                    
        except KeyboardInterrupt:
            logger.info("Stopped by user (Ctrl+C)")
        finally:
            logger.info(f"Total runtime: {time.time() - start_time:.1f}s")
            logger.info(f"Total loops: {loop_count}")
            self.redis_transfer.close()


def main():
    """Main function"""
    parser = argparse.ArgumentParser(
        description='Send full-body dance motion to Redis for robot control'
    )
    parser.add_argument('--host', default=REDIS_HOST, 
                       help=f'Redis server hostname (default: {REDIS_HOST})')
    parser.add_argument('--port', type=int, default=REDIS_PORT, 
                       help=f'Redis server port number (default: {REDIS_PORT})')
    parser.add_argument('--key', default=REDIS_KEY, 
                       help=f'Redis key to write (default: {REDIS_KEY})')
    parser.add_argument('--duration', type=float, default=None, 
                       help='Execution duration in seconds. If not specified, runs indefinitely')
    parser.add_argument('--frequency', type=int, default=50, 
                       help='Transmission frequency in Hz (default: 50Hz)')
    
    args = parser.parse_args()
    
    # Create DanceMotion instance
    dance = DanceMotion(host=args.host, port=args.port, redis_key=args.key)
    
    # Check Redis connection
    if not dance.redis_transfer.check_connection():
        logger.error(f"Failed to connect to Redis server {args.host}:{args.port}")
        dance.redis_transfer.close()
        sys.exit(1)
    
    # Execute dance motion
    dance.run(duration=args.duration, frequency=args.frequency)


if __name__ == "__main__":
    main()
