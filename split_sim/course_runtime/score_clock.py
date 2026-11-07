"""Sim-time freshness with source timestamp progress and separate stall watchdog."""
class TelemetryClock:
    def __init__(self):
        self.stamp = self.receipt_sim = self.receipt_wall = None
        self.reversed = False

    def observe(self, stamp_us, sim_t, wall_t):
        stamp = float(stamp_us)*1e-6
        if self.stamp is not None and stamp < self.stamp:
            self.reversed = True
            return
        if self.stamp is None or stamp > self.stamp:
            self.stamp,self.receipt_sim,self.receipt_wall=stamp,sim_t,wall_t

    def age(self, sim_t):
        if self.stamp is None or self.reversed: return float('inf')
        # This isolated PX4 SITL stamps messages in the Gazebo simulation epoch
        # (verified live). Never infer an epoch offset from first receipt: doing
        # that could bless a delayed old stream as fresh.
        if self.stamp > sim_t + .5: return float('inf')
        return max(0.,sim_t-self.receipt_sim,sim_t-self.stamp)

    def stalled(self, wall_t, limit=60.):
        return self.receipt_wall is None or wall_t-self.receipt_wall>limit
