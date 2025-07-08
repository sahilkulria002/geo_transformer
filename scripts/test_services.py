#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geo_transformer.srv import SetOrigin, GetOrigin, FromLL, ToLL

class ServiceTester(Node):
    def __init__(self):
        super().__init__('geo_service_tester')
        self.origin = [28.6139, 77.2090, 0.0]  # India Gate
        self.test_point = [28.6145, 77.2105, 0.0]  # Nearby point

        # Use the correct service names as advertised by the C++ node
        self.set_origin_client = self.create_client(SetOrigin, '/local_coordinate/set')
        self.get_origin_client = self.create_client(GetOrigin, '/local_coordinate/get')
        self.from_ll_client = self.create_client(FromLL, '/from_ll')
        self.to_ll_client = self.create_client(ToLL, '/to_ll')

        self.timer = self.create_timer(1.0, self.run_sequence)

    def run_sequence(self):
        self.timer.cancel()

        print("[SetOrigin] Waiting for service...")
        self.set_origin_client.wait_for_service()
        req = SetOrigin.Request()
        req.latitude, req.longitude, req.altitude = self.origin
        self.set_origin_client.call_async(req).add_done_callback(self.on_set_origin)

    def on_set_origin(self, future):
        print("[SetOrigin] Done")

        print("[GetOrigin] Waiting for service...")
        self.get_origin_client.wait_for_service()
        self.get_origin_client.call_async(GetOrigin.Request()).add_done_callback(self.on_get_origin)

    def on_get_origin(self, future):
        res = future.result()
        print(f"[GetOrigin] lat={res.latitude}, lon={res.longitude}, alt={res.altitude}")

        print("[FromLL] Waiting for service...")
        self.from_ll_client.wait_for_service()
        req = FromLL.Request()
        req.latitude, req.longitude, req.altitude = self.test_point
        self.from_ll_client.call_async(req).add_done_callback(self.on_from_ll)

    def on_from_ll(self, future):
        res = future.result()
        print(f"[FromLL] x={res.x}, y={res.y}, z={res.z}")

        print("[ToLL] Waiting for service...")
        self.to_ll_client.wait_for_service()
        req = ToLL.Request()
        req.x, req.y, req.z = res.x, res.y, res.z
        self.to_ll_client.call_async(req).add_done_callback(self.on_to_ll)

    def on_to_ll(self, future):
        res = future.result()
        print(f"[ToLL] lat={res.latitude}, lon={res.longitude}, alt={res.altitude}")

        # Round-trip check: compare input test_point with output from ToLL
        import math
        lat1, lon1, alt1 = self.test_point
        lat2, lon2, alt2 = res.latitude, res.longitude, res.altitude

        # Compute differences
        dlat = abs(lat1 - lat2)
        dlon = abs(lon1 - lon2)
        dalt = abs(alt1 - alt2)

        # Haversine formula for distance in meters (ignoring altitude)
        R = 6371000  # Earth radius in meters
        phi1 = math.radians(lat1)
        phi2 = math.radians(lat2)
        dphi = math.radians(lat2 - lat1)
        dlambda = math.radians(lon2 - lon1)
        a = math.sin(dphi/2)**2 + math.cos(phi1)*math.cos(phi2)*math.sin(dlambda/2)**2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
        horizontal_error = R * c
        vertical_error = dalt

        print(f"[RoundTrip] Horizontal error: {horizontal_error:.6f} m, Vertical error: {vertical_error:.6f} m")
        # Optionally, assert or warn if error is too large
        if horizontal_error < 1.0 and vertical_error < 0.1:
            print("[RoundTrip] ✅ Transformation is accurate (within 1 meter, 0.1m altitude)")
        else:
            print("[RoundTrip] ❌ Transformation error is too large!")
        rclpy.shutdown()


def main():
    print("Starting service tester...")
    rclpy.init()
    rclpy.spin(ServiceTester())

if __name__ == '__main__':
    main()
