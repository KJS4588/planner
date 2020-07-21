class OdomDouble {
	private:
		double x, y, z;

	public:
		OdomDouble(double x, double y, double z) {
			this->x = x;
			this->y = y;
			this->z = 0.0;
		}

		// getters
		double getX() {
			return this->x;
		}

		double getY() {
			return this->y;
		}

		double getZ() {
			return this->z;
		}
};
