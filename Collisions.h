#pragma once

#include <algorithm>

//! Collishi version 0.2.0

//! Here resides the actual mathematical core of the collision routines from the Shidacea engine
//! These are completely decoupled from any Ruby or SFML magic

//! Sadly, std::abs is not a constexpr, so this definition will provide one

template <class T> constexpr T constexpr_abs(T value) {

	return (value < 0.0f ? -value : value);

}

//! Helper function to check whether a fractional value is greater than zero without actually doing the division

template <class T> constexpr bool fraction_less_than_zero(T nominator, T denominator) {

	if (nominator == static_cast<T>(0)) return false;
	if (static_cast<bool>(nominator < static_cast<T>(0)) != static_cast<bool>(denominator < static_cast<T>(0))) return true;

	return false;

}

template <class T> constexpr bool fraction_between_zero_and_one(T nominator, T denominator) {

	if (fraction_less_than_zero(nominator, denominator)) return false;
	if (constexpr_abs(nominator) > constexpr_abs(denominator)) return false;

	return true;

}

template <class T> constexpr bool between(T value, T border_1, T border_2) {

	auto interval = std::minmax(border_1, border_2);

	if (value < interval.first) return false;
	if (value > interval.second) return false;

	return true;

}

template <class T> constexpr bool overlap(std::initializer_list<T> interval_1, std::initializer_list<T> interval_2) {

	auto interval_1_minmax = std::minmax(interval_1);
	auto interval_2_minmax = std::minmax(interval_2);

	if (interval_2_minmax.second < interval_1_minmax.first) return false;
	if (interval_1_minmax.second < interval_2_minmax.first) return false;

	return true;

}

template <class T> constexpr T sign_square(T x) {

	return (x < 0.0f ? -x * x : x * x);

}

//! Actual collision routines

constexpr bool collision_point_point(float x1, float y1, float x2, float y2) {

	//! Usually, this routine will yield false

	return (x1 == x2 && y1 == y2);

	return false;

}

constexpr bool collision_point_line(float x1, float y1, float x2, float y2, float dx2, float dy2) {

	//! The most useful check is to check whether the point has a normal component to the line
	//! If so, it is impossible for the point to intersect the line
	//! This case is also the most common one, so it is wise to check it first

	auto dx12 = x1 - x2;
	auto dy12 = y1 - y2;

	//! Check whether the cross product of the distance vector and the line vector is vanishing

	if (dx12 * dy2 != dy12 * dx2) return false;

	//! Otherwise, the point is on the infinite extension of the line
	//! Now, the point will be projected to the line
	//! If this projection value is smaller than 0, the point is not on the line
	//! If it is greater than the line end point projected on the line, it is also not on the line

	auto projection = dx12 * dx2 + dy12 * dy2;

	if (!between(projection, 0.0f, dx2 * dx2 + dy2 * dy2)) return false;

	return true;

}

constexpr bool collision_point_circle(float x1, float y1, float x2, float y2, float r2) {

	//! Simple check whether the point is inside the circle radius

	auto dx = x1 - x2;
	auto dy = y1 - y2;

	if (dx * dx + dy * dy > r2 * r2) return false;
	
	return true;

}

constexpr bool collision_point_box(float x1, float y1, float x2, float y2, float w2, float h2) {

	//! Literally the definition of an AABB

	if (x1 < x2) return false;
	if (y1 < y2) return false;
	if (x2 + w2 < x1) return false;
	if (y2 + h2 < y1) return false;
	
	return true;
}

constexpr bool collision_point_triangle(float x1, float y1, float x2, float y2, float sxa2, float sya2, float sxb2, float syb2) {

	//! Point coordinates relative to the first triangle point

	auto dx12 = x1 - x2;
	auto dy12 = y1 - y2;

	//! The next step consists of calculating the coordinates of the point formulated as linear combination of the two sides
	//! This yields the values u and v, which need to satisfy the following conditions: u >= 0, v >= 0, u + v <= 1

	auto nominator_u = dx12 * syb2 - dy12 * sxb2;
	auto denominator_u = sxa2 * syb2 - sxb2 * sya2;

	if (!fraction_between_zero_and_one(nominator_u, denominator_u)) return false;

	auto nominator_v = dx12 * sya2 - dy12 * sxa2;
	auto denominator_v = -denominator_u;

	if (!fraction_between_zero_and_one(nominator_v, denominator_v)) return false;

	//! The condition u + v <= 1 has one caveat, namely the case that the nominators of u and -v have different signs
	//! Therefore, the check for u + v >= 0 needs to be done explicitely again
	//! Furthermore, the denominator of u will be taken here as the new denominator

	auto nominator_u_v = nominator_u - nominator_v;

	if (!fraction_between_zero_and_one(nominator_u_v, denominator_u)) return false;

	//! If all values are inside the appropriate ranges, the point is inside the triangle

	return true;

}

constexpr bool collision_line_line(float x1, float y1, float dx1, float dy1, float x2, float y2, float dx2, float dy2) {

	//! This algorithm is an extension of point/line collisions
	//! First, the cross product of the two lines will be calculated
	//! If it is vanishing, the lines are both on their respective infinite extensions
	//! In that case, if the start points of one lines lies on the other line, they intersect
	//! Checking the end points is not necessary here
	
	auto cross_term = dx2 * dy1 - dy2 * dx1;

	if (cross_term == 0.0f) {

		if (collision_point_line(x1, y1, x2, y2, dx2, dy2)) return true;
		if (collision_point_line(x2, y2, x1, y1, dx1, dy1)) return true;
	
	}

	//! The lines have a normal component, so they have only up to one intersection point
	//! Now, the separating axis theorem can be applied to the situation
	//! Both lines are then projected on the other line normal
	//! If the two projections (start and end point) change their sign, they intersect the other line
	//! If the projection interval doesn't contain 0, an intersection is excluded completely

	auto y21 = y2 - y1;
	auto x21 = x2 - x1;

	auto projection_2_on_n1 = y21 * dx1 - x21 * dy1;

	if (static_cast<bool>(projection_2_on_n1 < 0.0f) == static_cast<bool>(projection_2_on_n1 < cross_term)) return false;

	auto projection_1_on_n2 = x21 * dy2 - y21 * dx2;

	if (static_cast<bool>(projection_1_on_n2 < 0.0f) == static_cast<bool>(projection_1_on_n2 < -cross_term)) return false;

	return true;
}

constexpr bool collision_line_circle(float x1, float y1, float dx1, float dy1, float x2, float y2, float r2) {

	//! This algorithm is a direct implementation of the separating axis theorem
	//! If there is axis at which the projections of both objects do not overlap, they don't intersect

	//! Calculate difference coordinates
	
	auto x21 = x2 - x1;
	auto y21 = y2 - y1;

	auto r2_squared = r2 * r2;

	//! Project the circle directly on the line and check whether there is a gap or not
	//! This involves a small trick, since the projection of the circle itself is not trivial on non-axis-aligned lines
	//! Normally, this projection would be r2 * sqrt(|line|), where line is the vector to which the circle will be projected
	//! The test would then be whether 0 (the projection of the line on itself) is between proj_circle_normal +/- r2 * sqrt(|line|)
	//! The square root can be removed by taking the square of the whole test, but conserving the signs of each side
	//! Then, only a square instead of a square root is necessary, speeding up the procedure
	//! Finally, both sides can be substracted by proj_circle_normal

	auto proj_circle_normal = y21 * dx1 - x21 * dy1;
	auto proj_circle_normal_max = r2_squared * (dx1 * dx1 + dy1 * dy1);

	if (!between(sign_square(proj_circle_normal), -proj_circle_normal_max, proj_circle_normal_max)) return false;

	//! Now check the closest point on the line and take the difference between it and the circle midpoint as a new axis to test
	//! If this test is done, no other axes need to be tested

	auto x2d1 = x21 - dx1;
	auto y2d1 = y21 - dy1;

	auto distance_1_2 = x21 * x21 + y21 * y21;
	auto distance_d_2 = x2d1 * x2d1 + y2d1 * y2d1;

	if (distance_1_2 < distance_d_2) {

		//! Start point is closer to circle

		auto p1 = sign_square(distance_1_2);
		auto p2 = sign_square(distance_1_2 - dx1 * x21 - dy1 * y21);

		auto proj_r2_squared = r2_squared * (x21 * x21 + y21 * y21);

		if (!overlap({ p1, p2 }, { -proj_r2_squared, proj_r2_squared })) return false;

	} else {

		//! End point is closer to circle

		auto p1 = sign_square(distance_d_2);
		auto p2 = sign_square(distance_1_2 - dx1 * x21 - dy1 * y21);

		auto proj_r2_squared = r2_squared * (x2d1 * x2d1 + y2d1 * y2d1);

		if (!overlap({ p1, p2 }, { -proj_r2_squared, proj_r2_squared })) return false;

	}

	return true;

}

constexpr bool collision_line_box(float x1, float y1, float dx1, float dy1, float x2, float y2, float w2, float h2) {

	//! First check whether any end point lies inside the box

	if (collision_point_box(x1, y1, x2, y2, w2, h2)) return true;
	if (collision_point_box(x1 + dx1, y1 + dy1, x2, y2, w2, h2)) return true;
	
	//! If this is not the case, check each axis for an intersection inside the rectangle

	//! These value can all be calculated in beforehand, since 3/4 of them will be already used in the first check
	//! This will make the code much less complicated

	//! Nominators of the line parameter

	auto nominator_x_neg = x2 - x1;
	auto nominator_x_pos = x2 + w2 - x1;
	auto nominator_y_neg = y2 - y1;
	auto nominator_y_pos = y2 + h2 - y1;

	//! These terms can be obtained by inserting the line parameter for one coordinate into the intersection equation
	//! This is only a mathematical trick to avoid divisions here

	auto nom_x_neg_dy = nominator_x_neg * dy1;
	auto nom_x_pos_dy = nominator_x_pos * dy1;
	auto nom_y_neg_dx = nominator_y_neg * dx1;
	auto nom_y_pos_dx = nominator_y_pos * dx1;

	//! Check whether the y coordinate of the intersection point with the left AABB side is actually inside the AABB
	//! The following checks will repeat this procedure for the other sides

	if ((nom_x_neg_dy >= nom_y_neg_dx) && (nom_x_neg_dy <= nom_y_pos_dx)) {

		//! The case of a vanishing dx1 should not occur, but even then, the next check will rule it out definitely
		//! Here, the line parameter of the intersection point will be checked for its sign
		//! If it is smaller than 0 or greater than 1, the line segment does not touch the AABB at this side
		//! This can again be checked by comparing the signs of the nominator and the denominator
		//! We still need to check the other sides, however

		if (fraction_between_zero_and_one(nominator_x_neg, dx1)) return true;

	}

	//! Check right side

	if ((nom_x_pos_dy >= nom_y_neg_dx) && (nom_x_pos_dy <= nom_y_pos_dx)) {

		//! The line got shifted in its coordinates, so a new line parameter check is necessary

		if (fraction_between_zero_and_one(nominator_x_pos, dx1)) return true;

	}

	//! Check bottom side

	if ((nom_y_neg_dx >= nom_x_neg_dy) && (nom_y_neg_dx <= nom_x_pos_dy)) {

		if (fraction_between_zero_and_one(nominator_y_neg, dy1)) return true;

	}

	//! Check top side

	if ((nom_y_pos_dx >= nom_x_neg_dy) && (nom_y_pos_dx <= nom_x_pos_dy)) {

		if (fraction_between_zero_and_one(nominator_y_pos, dy1)) return true;

	}

	return false;

}

constexpr bool collision_line_triangle(float x1, float y1, float dx1, float dy1, float x2, float y2, float sxa2, float sya2, float sxb2, float syb2) {
	
	//! This function is another application of the separating axis theorem
	//! First, the distances between the line starting point and the three triangle vertices will be calculated

	auto x21 = x2 - x1;
	auto y21 = y2 - y1;

	auto xa1 = x21 + sxa2;
	auto ya1 = y21 + sya2;

	auto xb1 = x21 + sxb2;
	auto yb1 = y21 + syb2;

	//! Now, all three vertices will be projected on the line

	auto projection_2_on_n1 = y21 * dx1 - x21 * dy1;
	auto projection_a_on_n1 = ya1 * dx1 - xa1 * dy1;
	auto projection_b_on_n1 = yb1 * dx1 - xb1 * dy1;

	//! If no sign change occurs between all three projections, the triangle doesn't intersect the line

	auto p2_n1_negative = static_cast<short>(projection_2_on_n1 < 0.0f);
	auto pa_n1_negative = static_cast<short>(projection_a_on_n1 < 0.0f);
	auto pb_n1_negative = static_cast<short>(projection_b_on_n1 < 0.0f);

	if (p2_n1_negative + pa_n1_negative + pb_n1_negative == 3) return false;

	//! Now, the line needs to be projected on each triangle side
	//! This time, if both line points are outside of the interval between 0 and the opposite vertex, no intersection happens

	auto projection_1_on_na = x21 * sya2 - y21 * sxa2;
	auto projection_d_on_na = dy1 * sxa2 - dx1 * sya2;
	auto projection_b_on_na = syb2 * sxa2 - sxb2 * sya2;

	if (!overlap({ projection_1_on_na, projection_1_on_na + projection_d_on_na }, { 0.0f, projection_b_on_na })) return false;

	//! This needs to be repeated for the other given triangle side

	auto projection_1_on_nb = x21 * syb2 - y21 * sxb2;
	auto projection_d_on_nb = dy1 * sxb2 - dx1 * syb2;
	auto projection_a_on_nb = -projection_b_on_na;

	if (!overlap({ projection_1_on_nb, projection_1_on_nb + projection_d_on_nb }, { 0.0f, projection_a_on_nb })) return false;

	//! The last line is the difference vector between the vertices A and B

	auto sxc2 = sxb2 - sxa2;
	auto syc2 = syb2 - sya2;

	auto projection_1_on_nc = xa1 * syc2 - ya1 * sxc2;
	auto projection_d_on_nc = dy1 * sxc2 - dx1 * syc2;
	auto projection_2_on_nc = projection_b_on_na;

	if (!overlap({ projection_1_on_nc, projection_1_on_nc + projection_d_on_nc }, { 0.0f, projection_2_on_nc })) return false;

	return true;

}

constexpr bool collision_circle_circle(float x1, float y1, float r1, float x2, float y2, float r2) {

	//! Simple generalization of point/circle

	auto dx = x1 - x2;
	auto dy = y1 - y2;

	auto combined_radius = r1 + r2;

	if (dx * dx + dy * dy > combined_radius * combined_radius) return false;

	return true;

}

constexpr bool collision_circle_box(float x1, float y1, float r1, float x2, float y2, float w2, float h2) {

	//! This algorithm makes use of the separating axis theorem (SAT)
	//! Essentially, the circle is projected onto both cardinal axes

	//! Projections of the circle onto the axes

	auto dxp = (x2 + w2 - x1);
	auto dyp = (y2 + h2 - y1);
	auto dxm = (x2 - x1);
	auto dym = (y2 - y1);

	//! Check for intersection of the circle projections with the AABB projections

	if (!overlap({ dxm, dxp }, { -r1, r1 })) return false;
	if (!overlap({ dym, dyp }, { -r1, r1 })) return false;

	//! Calculated distances to circle to determine closest vertex

	auto dxp2 = dxp * dxp;
	auto dxm2 = dxm * dxm;
	auto dyp2 = dyp * dyp;
	auto dym2 = dym * dym;

	auto dxp2yp2 = dxp2 + dyp2;
	auto dyp2xm2 = dyp2 + dxm2;
	auto dxm2ym2 = dxm2 + dym2;
	auto dym2xp2 = dym2 + dxp2;

	//! Find out the vertex by brute forcing

	float min_dist = dxp2yp2;
	float vx = dxp;
	float vy = dyp;

	if (dyp2xm2 < min_dist) {
		
		min_dist = dyp2xm2;
		vx = dxm;
		vy = dyp;
	
	}

	if (dxm2ym2 < min_dist) {

		min_dist = dxm2ym2;
		vx = dxm;
		vy = dym;

	}

	if (dym2xp2 < min_dist) {

		min_dist = dym2xp2;
		vx = dxp;
		vy = dym;

	}

	//! Project AABB on difference vector with circle midpoint defined as zero

	auto proj_v_pp = sign_square(dxp * vx + dyp * vy);
	auto proj_v_pm = sign_square(dxp * vx + dym * vy);
	auto proj_v_mp = sign_square(dxm * vx + dyp * vy);
	auto proj_v_mm = sign_square(dxm * vx + dym * vy);

	auto proj_r1_squared = r1 * r1 * (vx * vx + vy * vy);

	if (!overlap({ proj_v_pp, proj_v_pm, proj_v_mp, proj_v_mm }, { -proj_r1_squared, proj_r1_squared })) return false;

	return true;

}

constexpr bool collision_circle_triangle(float x1, float y1, float r1, float x2, float y2, float sxa2, float sya2, float sxb2, float syb2) {

	//! This test is similar to circle/box, but the checked axes are different
	//! The first three axes are the normals of the triangle edges
	//! The procedure here is fully according to the separating axis theorem again

	auto dx = x1 - x2;
	auto dy = y1 - y2;

	auto r1_squared = r1 * r1;
	auto cross_term = sxa2 * syb2 - sxb2 * sya2;

	//! Check edge a

	auto proj_x1_a = dy * sxa2 - dx * sya2;
	auto proj_r1_a_squared = r1_squared * (sxa2 * sxa2 + sya2 * sya2);
	
	if (!overlap({ sign_square(-proj_x1_a), sign_square(cross_term - proj_x1_a) }, { -proj_r1_a_squared, proj_r1_a_squared })) return false;

	//! Check edge b

	auto proj_x1_b = dy * sxb2 - dx * syb2;
	auto proj_r1_b_squared = r1_squared * (sxb2 * sxb2 + syb2 * syb2);

	if (!overlap({ sign_square(-proj_x1_b), sign_square(-cross_term - proj_x1_b) }, { -proj_r1_b_squared, proj_r1_b_squared })) return false;

	//! Check edge c (the one spanned by vertices A and B)

	auto sxc2 = sxb2 - sxa2;
	auto syc2 = syb2 - sya2;
	
	//! Normally, the following term also contains the cross term, but the projection of the base vertex on this line lets this term vanish
	//! Therefore, it is possible to drop it completely from the following comparison

	auto proj_x1_c = dy * (sxb2 - sxa2) - dx * (syb2 - sya2);
	auto proj_r1_c_squared = r1_squared * (sxc2 * sxc2 + syc2 * syc2);

	if (!overlap({ sign_square(-proj_x1_c), sign_square(proj_x1_c) }, { -proj_r1_c_squared, proj_r1_c_squared })) return false;

	//! All normal checks were inconclusive, so the line from the circle to the closest vertex is required for a last test
	//! This is again the same principle as for circle/box

	float min_dist = dx * dx + dy * dy;
	float vx = -dx;
	float vy = -dy;

	auto dxa = dx - sxa2;
	auto dya = dy - sya2;

	auto dxb = dx - sxb2;
	auto dyb = dy - syb2;

	auto da_norm = dxa * dxa + dya * dya;
	auto db_norm = dxb * dxb + dyb * dyb;

	if (da_norm < min_dist) {

		min_dist = da_norm;
		vx = -dxa;
		vy = -dya;

	}

	if (db_norm < min_dist) {

		min_dist = db_norm;
		vx = -dxb;
		vy = -dyb;

	}

	//! Projection of the triangle on the line

	auto proj_2_0_v = sign_square(-dx * vx - dy * vy);
	auto proj_2_a_v = sign_square(-dxa * vx - dya * vy);
	auto proj_2_b_v = sign_square(-dxb * vx - dyb * vy);

	auto proj_r_v_squared = r1_squared * min_dist;

	if (!overlap({ proj_2_0_v, proj_2_a_v, proj_2_b_v }, { -proj_r_v_squared, proj_r_v_squared })) return false;

	return true;

}

constexpr bool collision_box_box(float x1, float y1, float w1, float h1, float x2, float y2, float w2, float h2) {

	//! Simple generalization of point/box

	if (x1 + w1 < x2) return false;
	if (y1 + h1 < y2 ) return false;
	if (x2 + w2 < x1) return false;
	if (y2 + h2 < y1) return false;

	return true;

}

#ifndef IGNORE_STATIC_ASSERTIONS

//! Compile time assertions to check some test cases
//! Please submit a bug report if one of these fails
//! Also please submit a bug report if you encounter a case which fails and can be reproduced using an assertion

static_assert(true == fraction_less_than_zero(-1.0, 3.0));
static_assert(true == fraction_less_than_zero(1.0, -3.0));
static_assert(false == fraction_less_than_zero(0.0, 3.0));
static_assert(false == fraction_less_than_zero(0.0, -3.0));
static_assert(false == fraction_less_than_zero(1.0, 3.0));
static_assert(false == fraction_less_than_zero(-1.0, -3.0));

static_assert(false == fraction_between_zero_and_one(-1.0, 3.0));
static_assert(false == fraction_between_zero_and_one(1.0, -3.0));
static_assert(true == fraction_between_zero_and_one(0.0, 3.0));
static_assert(true == fraction_between_zero_and_one(0.0, -3.0));
static_assert(true == fraction_between_zero_and_one(1.0, 3.0));
static_assert(true == fraction_between_zero_and_one(-1.0, -3.0));
static_assert(false == fraction_between_zero_and_one(3.0, 1.0));
static_assert(false == fraction_between_zero_and_one(-3.0, 1.0));
static_assert(false == fraction_between_zero_and_one(-3.0, -1.0));

static_assert(true == overlap({ 1, 3, 4 }, { 2, 1 }));
static_assert(false == overlap({ 1, 3, 4 }, { 6, 5 }));
static_assert(false == overlap({ -1, 6 }, { -3 }));
static_assert(true == overlap({ -1, 6 }, { 3 }));
static_assert(true == overlap({ -1, 6 }, { -1 }));
static_assert(true == overlap({ -1, 6 }, { 6 }));

static_assert(false == collision_point_point(1.0f, 2.0f,     3.0f, 4.0f));
static_assert(true == collision_point_point(1.0f, 9.0f,     1.0f, 9.0f));

static_assert(true == collision_point_line(0.2f, 0.2f,     0.0f, 0.0f, 1.0f, 1.0f));
static_assert(false == collision_point_line(0.2f, 0.3f,     0.0f, 0.0f, 1.0f, 1.0f));
static_assert(true == collision_point_line(1.0f, 0.0f,     0.0f, 0.0f, 1.0f, 0.0f));
static_assert(true == collision_point_line(1.0f, 0.0f,      1.0f, 0.0f, 1.0f, 0.0f));
static_assert(false == collision_point_line(1.0f, 0.0f,     1.1f, 0.0f, 1.0f, 0.0f));

static_assert(true == collision_point_circle(2.0f, 3.0f,     4.0f, 5.0f, 3.0f));

static_assert(true == collision_point_box(-3.0f, -5.0f,     -7.0f, -8.0f, 20.0f, 18.0f));

static_assert(true == collision_point_triangle(0.0f, 0.0f,     0.0f, 0.2f, 3.0f, -1.0f, -3.0f, -1.0f));
static_assert(false == collision_point_triangle(0.0f, 0.0f,     0.0f, 0.2f, 3.0f, 1.0f, -3.0f, 1.0f));

static_assert(true == collision_line_line(0.0f, 0.0f, 1.0f, 1.0f,     0.0f, 1.0f, 1.0f, -1.0f));
static_assert(false == collision_line_line(0.0f, 0.0f, 1.0f, 0.0f,     1.1f, -1.0f, 0.0f, 2.0f));
static_assert(true == collision_line_line(0.0f, 0.0f, 1.0f, 0.0f,     0.9f, -1.0f, 0.0f, 2.0f));
static_assert(false == collision_line_line(0.0f, 0.0f, 1.0f, 1.0f,     0.0f, 0.1f, 1.0f, 1.0f));

static_assert(true == collision_line_line(0.0f, 0.0f, 1.0f, 0.0f,     1.0f, 0.0f, 1.0f, 0.0f));
static_assert(false == collision_line_line(0.0f, 0.0f, 1.0f, 0.0f,     1.1f, 0.0f, 1.0f, 0.0f));
static_assert(false == collision_line_line(1.1f, 0.0f, 1.0f, 0.0f,     0.0f, 0.0f, 1.0f, 0.0f));

static_assert(true == collision_line_circle(1.0f, 1.0f, 8.0f, 8.0f,     -3.0f, -3.0f, 100.0f));
static_assert(true == collision_line_circle(1.0f, 1.0f, 8.0f, 8.0f,      4.0f, 4.0f, 0.1f));
static_assert(false == collision_line_circle(1.0f, 1.0f, 8.0f, 8.0f,      10.0f, 10.0f, 1.4f));
static_assert(true == collision_line_circle(1.0f, 1.0f, 8.0f, 8.0f,      10.0f, 10.0f, 1.5f));

static_assert(true == collision_line_box(3.0f, 2.0f, 8.0f, 11.0f,     0.0f, 1.0f, 10.0f, 10.0f));
static_assert(false == collision_line_box(11.0f, 0.0f, 11.0f, 13.0f,     0.0f, 1.0f, 10.0f, 10.0f));
static_assert(true == collision_line_box(1.0f, 1.0f, 7.0f, 7.0f,     2.0f, 2.0f, 4.0f, 4.0f));

static_assert(true == collision_line_triangle(3.0f, 0.0f, 0.0f, 2.0f,     2.0f, 1.0f, -1.0f, 3.0f, 2.0f, 1.0f));
static_assert(false == collision_line_triangle(2.0f, 4.0f, 2.0f, 0.0f,     2.0f, 1.0f, -1.0f, 3.0f, 2.0f, 1.0f));
static_assert(true == collision_line_triangle(2.0f, 1.0f, -1.0f, 3.0f,     2.0f, 1.0f, -1.0f, 3.0f, 2.0f, 1.0f));
static_assert(true == collision_line_triangle(2.0f, 1.0f, 2.0f, 1.0f,     2.0f, 1.0f, -1.0f, 3.0f, 2.0f, 1.0f));

static_assert(true == collision_circle_box(1.0f, -3.0f, 4.0f,     -5.0f, -4.0f, 10.0f, 8.0f));
static_assert(true == collision_circle_box(1.0f, -3.0f, 1.0f,     -5.0f, -2.0f, 10.0f, 4.0f));
static_assert(false == collision_circle_box(1.0f, -3.0f, 0.9f,     -5.0f, -2.0f, 10.0f, 4.0f));
static_assert(true == collision_circle_box(2.0f, 1.0f, 0.1f,     -2.0f, -2.0f, 4.0f, 4.0f));
static_assert(false == collision_circle_box(3.0f, 3.0f, 1.0f,     -2.0f, -2.0f, 4.0f, 4.0f));
static_assert(true == collision_circle_box(3.0f, 3.0f, 1.5f,     -2.0f, -2.0f, 4.0f, 4.0f));
static_assert(true == collision_circle_box(3.0f, 3.0f, 2.0f,     -2.0f, -2.0f, 4.0f, 4.0f));

static_assert(false == collision_circle_triangle(5.0f, 5.0f, 3.0f,     3.0f, 2.0f, -1.0f, -5.0f, -5.0f, -1.0f));
static_assert(true == collision_circle_triangle(0.0f, 0.0f, 1.0f,     3.0f, 2.0f, -1.0f, -5.0f, -5.0f, -1.0f));
static_assert(true == collision_circle_triangle(5.0f, 5.0f, 4.0f,     3.0f, 2.0f, -1.0f, -5.0f, -5.0f, -1.0f));

static_assert(true == collision_box_box(-2.0f, -2.0f, 6.0f, 8.0f,     2.5f, 5.5f, 4.0f, 4.0f));
static_assert(false == collision_box_box(-2.0f, -2.0f, 6.0f, 8.0f,     3.1f, 6.1f, 2.8f, 2.8f));

#endif