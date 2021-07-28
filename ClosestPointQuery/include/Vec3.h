#pragma once
#include <intrin.h>
#include <limits>

namespace math {

	class alignas(16) Vec3 {
	private:
		__m128 _data;
	public:
		Vec3() : _data{ _mm_setr_ps(0.f, 0.f, 0.f, 0.f) } {}
		Vec3(float val) : _data{ _mm_set_ps1(val) } {}
		Vec3(float x, float y, float z) : _data{ _mm_setr_ps(x, y, z, 0.f) } {}
		Vec3(const Vec3& other) : _data{ other._data } {}
		~Vec3() = default;
		Vec3& operator=(const Vec3& other) { if (this == &other) return *this; _data = other._data; return *this; }
		Vec3 operator+(const Vec3& other) const { return Vec3(_mm_add_ps(_data, other._data)); }
		Vec3 operator-(const Vec3& other) const { return Vec3(_mm_sub_ps(_data, other._data)); }
		Vec3 operator*(const Vec3& other) const { return Vec3(_mm_mul_ps(_data, other._data)); }
		Vec3 operator/(const Vec3& other) const { return Vec3(_mm_div_ps(_data, other._data)); }
		Vec3 operator-() const { return Vec3(_mm_sub_ps(_mm_set1_ps(0.f), _data)); }
		Vec3 operator*(float scalar) const { return Vec3(_mm_mul_ps(_data, _mm_set_ps1(scalar))); }
		bool operator==(const Vec3& other) const { return _mm_movemask_ps(_mm_cmpeq_ps(_data, other._data)) & 0xE; }
		bool operator!=(const Vec3& other) const { return !(*this == other); }
		const float& operator[](size_t idx) const { return _data.m128_f32[idx]; }
		bool nearly_zero() const { return length() < std::numeric_limits<float>::epsilon(); }
		float x() const { return _data.m128_f32[0]; }
		float y() const { return _data.m128_f32[1]; }
		float z() const { return _data.m128_f32[2]; }
		float dot(const Vec3& other) const { return _mm_dp_ps(_data, other._data, 0x78).m128_f32[3]; }
		float length() const { return sqrtf(_mm_dp_ps(_data, _data, 0x78).m128_f32[3]); }
		float length2() const { return _mm_dp_ps(_data, _data, 0x78).m128_f32[3]; }
		float distance(const Vec3& other) const { return (*this - other).length(); }
		float distance2(const Vec3& other) const { return (*this - other).length2(); }
		Vec3 min(const Vec3& other) const { return Vec3(_mm_min_ps(_data, other._data)); }
		Vec3 max(const Vec3& other) const { return Vec3(_mm_max_ps(_data, other._data)); }
		Vec3 normalize() const {
			const __m128 length = _mm_sqrt_ps(_mm_dp_ps(_data, _data, 0x77));
			return Vec3(_mm_mul_ps(_data, _mm_div_ps(_mm_set_ps1(1.0f), length)));
		}
		Vec3 cross(const Vec3& other) const {
			const __m128 tmp0 = _mm_shuffle_ps(_data, _data, _MM_SHUFFLE(3, 0, 2, 1));
			const __m128 tmp1 = _mm_shuffle_ps(other._data, other._data, _MM_SHUFFLE(3, 1, 0, 2));
			const __m128 tmp2 = _mm_shuffle_ps(_data, _data, _MM_SHUFFLE(3, 1, 0, 2));
			const __m128 tmp3 = _mm_shuffle_ps(other._data, other._data, _MM_SHUFFLE(3, 0, 2, 1));
			return _mm_sub_ps(
				_mm_mul_ps(tmp0, tmp1),
				_mm_mul_ps(tmp2, tmp3)
			);
		}
	private:
		Vec3(const __m128& _data) : _data{ _data } {}
	};

}; // namespace math