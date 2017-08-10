# We designed a suitable crop row detection performance measure,
# referred to as Crop Row Detection Accuracy (CRDA), which is defined as
# CRDA is computed by matching horizontal coordinates uv,i of
# crop rows obtained by the evaluated method to the corresponding
# ground truth values u
# v;i according to the matching score (21) and
# computing the average of the matching scores for all image rows.
# A matching score s computed by (21) represents a value from the
# interval [0,1]. In the case of a perfect match, i.e., if uv;i ¼ u
# v;i
# ,
# function s returns the maximum value 1. If the difference between
# a value uv,i and the corresponding ground truth value u
# v;i is
# u
# v;i uv;i
# 
# 
# 
# 
# 
# 4σd, function s returns 0. The shape of the function s is
# shown in Fig. 15. In the experiments presented in this section,
# CRDA is computed with σ¼0.1, which means that the matching
# score is greater than zero only if the horizontal distance between a
# detected crop row and the corresponding ground truth curve is
# less than 10% of the distance between adjacent crop rows. In
# general, value σ depends on the desired accuracy needed for safe
# guidance of an agricultural machine.
# For all evaluated methods, three crop rows are compared to the
# ground truth data, i.e., parameter m in (20) is 3. In cases where
# more than three crop rows are visible starting from the bottom of
# the image, matching the detected crop rows to the ground truth
# data can be ambiguous. Therefore, ground truth values u
# v;i are
# generated for nine adjacent crop rows and CRDA is computed for
# each three adjacent ground truth curves. The maximum of the
# obtained seven CRDA values is considered as the final performance
# measure for a particular image. CRDA values obtained by the
# TMGEM method are given below each subimage in Fig. 13.


sigma = 0.1


def s(u_star, u, d):
    datval = (u_star - u) / (sigma * d) ** 2
    return max(1 - (datval - u), 0)


total = 0
for v in range(v0, h - 1):
    for i in range(1, m):
        total += s(u_vi_star, u_vi, d_v_star)
