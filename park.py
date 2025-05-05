import sys
import glob
import os
import time
import numpy as np
import cv2

# Set CARLA .egg path (adjust if needed)
carla_path = r"C:\Users\sruja\Downloads\CARLA_0.9.11 (1)\WindowsNoEditor\PythonAPI\carla\dist"
sys.path.append(glob.glob(f"{carla_path}/carla-*%d.%d-%s.egg" % (
    sys.version_info.major, sys.version_info.minor,
    "win-amd64" if os.name == "nt" else "linux-x86_64"))[0])

import carla

# Parking config
invert_steering_point = -7.8
parked_locations = [
    carla.Transform(carla.Location(x=60.4, y=-10.62, z=0.05), carla.Rotation(yaw=180)),
    carla.Transform(carla.Location(x=47.0, y=-10.62, z=0.05), carla.Rotation(yaw=180))
]


class CarlaParkVehicle:
    def __init__(self):
        self.actor_list = []
        try:
            self.client = carla.Client('localhost', 2000)
            self.client.set_timeout(10.0)
            self.world = self.client.get_world()

            # Enable synchronous mode
            settings = self.world.get_settings()
            settings.synchronous_mode = True
            settings.fixed_delta_seconds = 0.05
            self.world.apply_settings(settings)

            blueprint_library = self.world.get_blueprint_library()

            # Spawn ego vehicle
            bp = blueprint_library.filter('vehicle.tesla.model3')[0]
            init_pos = carla.Transform(carla.Location(x=47.7, y=-7.62, z=0.05), carla.Rotation(yaw=180))
            self.vehicle = self.world.spawn_actor(bp, init_pos)
            self.actor_list.append(self.vehicle)

            # Spawn parked vehicles
            for pos in parked_locations:
                v = self.world.spawn_actor(bp, pos)
                self.actor_list.append(v)

            # Attach RGB camera
            camera_bp = blueprint_library.find('sensor.camera.rgb')
            camera_bp.set_attribute("image_size_x", "800")
            camera_bp.set_attribute("image_size_y", "600")
            camera_bp.set_attribute("fov", "90")
            camera_transform = carla.Transform(carla.Location(x=1.5, z=2.4))
            self.camera = self.world.spawn_actor(camera_bp, camera_transform, attach_to=self.vehicle)
            self.actor_list.append(self.camera)
            self.camera.listen(lambda image: self.process_image(image))

            # Spectator view
            spectator = self.world.get_spectator()
            spectator.set_transform(carla.Transform(
                carla.Location(x=55.0, y=-15.0, z=10.0),
                carla.Rotation(pitch=-30, yaw=0, roll=0)))

        except Exception as e:
            print(f"Error during initialization: {e}")
            self.destroy()
            raise

    def process_image(self, image):
        img = np.frombuffer(image.raw_data, dtype=np.uint8)
        img = img.reshape((image.height, image.width, 4))
        bgr = img[:, :, :3]

        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        _, binary = cv2.threshold(gray, 200, 255, cv2.THRESH_BINARY)

        contours, _ = cv2.findContours(binary, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        for cnt in contours:
            approx = cv2.approxPolyDP(cnt, 0.05 * cv2.arcLength(cnt, True), True)
            if len(approx) == 4 and cv2.contourArea(cnt) > 800:
                x, y, w, h = cv2.boundingRect(cnt)
                cv2.rectangle(bgr, (x, y), (x + w, y + h), (0, 255, 0), 2)

        cv2.imshow("Camera View - Parking Detection", bgr)
        cv2.waitKey(1)

    def move_to_init_parking(self):
        print("Moving to parking start position...")
        while self.vehicle.get_location().x > 50:
            self.vehicle.apply_control(carla.VehicleControl(throttle=0.4, brake=0.0))
            self.world.tick()
            time.sleep(0.05)

        print("Parking spot found!")
        self.vehicle.apply_control(carla.VehicleControl(throttle=0.0, brake=1.0))
        time.sleep(2)

    def park(self):
        print("Starting parking maneuver...")

        while True:
            self.vehicle.apply_control(
                carla.VehicleControl(throttle=0.3, steer=0.6, brake=0.0, reverse=True))
            self.world.tick()
            time.sleep(0.1)
            if self.vehicle.get_location().y < invert_steering_point:
                break

        while self.vehicle.get_location().y < invert_steering_point:
            self.vehicle.apply_control(
                carla.VehicleControl(throttle=0.2, steer=-0.6, brake=0.0, reverse=True))
            self.world.tick()
            time.sleep(0.1)
            if abs(self.vehicle.get_transform().rotation.yaw) > 178:
                print("Vehicle aligned with parking space")
                self.vehicle.apply_control(
                    carla.VehicleControl(throttle=0.0, steer=0.0, brake=1.0, reverse=True))
                break

        self.vehicle.apply_control(carla.VehicleControl(throttle=0.0, brake=1.0))
        time.sleep(0.5)

        print("Completing parking...")
        self.vehicle.apply_control(
            carla.VehicleControl(throttle=0.5, steer=0.0, brake=0.0, reverse=False))
        time.sleep(1.2)

        self.vehicle.apply_control(carla.VehicleControl(throttle=0.0, brake=1.0))
        time.sleep(0.5)
        print("Parking complete!")

    def destroy(self):
        print('Destroying actors...')
        for actor in self.actor_list:
            if actor.is_alive:
                actor.destroy()
        if hasattr(self, 'camera') and self.camera.is_alive:
            self.camera.stop()
            self.camera.destroy()
        cv2.destroyAllWindows()
        print('Done.')

    def run(self):
        try:
            self.move_to_init_parking()
            self.park()
            while True:
                self.world.tick()
                time.sleep(0.1)
        except KeyboardInterrupt:
            print("Simulation stopped by user")


if __name__ == "__main__":
    ego_vehicle = None
    try:
        ego_vehicle = CarlaParkVehicle()
        ego_vehicle.run()
    except Exception as e:
        print(f"Runtime error: {e}")
    finally:
        if ego_vehicle is not None:
            ego_vehicle.destroy()
