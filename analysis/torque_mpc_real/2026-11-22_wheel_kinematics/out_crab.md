/home/smoc/.local/lib/python3.12/site-packages/numpy/lib/function_base.py:1242: RuntimeWarning: divide by zero encountered in divide
  a = -(dx2)/(dx1 * (dx1 + dx2))
/home/smoc/.local/lib/python3.12/site-packages/numpy/lib/function_base.py:1243: RuntimeWarning: divide by zero encountered in divide
  b = (dx2 - dx1) / (dx1 * dx2)
/home/smoc/.local/lib/python3.12/site-packages/numpy/lib/function_base.py:1244: RuntimeWarning: divide by zero encountered in divide
  c = dx1 / (dx2 * (dx1 + dx2))
/home/smoc/.local/lib/python3.12/site-packages/numpy/lib/function_base.py:1250: RuntimeWarning: invalid value encountered in add
  out[tuple(slice1)] = a * f[tuple(slice2)] + b * f[tuple(slice3)] + c * f[tuple(slice4)]
/home/smoc/.local/lib/python3.12/site-packages/numpy/lib/function_base.py:1266: RuntimeWarning: divide by zero encountered in scalar divide
  out[tuple(slice1)] = (f[tuple(slice2)] - f[tuple(slice3)]) / dx_n
| 묶음 | 표본 | β_SLAM 평균 ° | β_바퀴 평균 ° | β_SLAM − β_바퀴 중앙값 ° [사분위] |
|---|---|---|---|---|
| 좌회전 ω>0.3 | 2245 | +2.46 | +1.94 | +0.90 [-1.23~+2.72] |
| 우회전 ω<−0.3 | 2246 | -0.17 | -4.88 | +4.40 [+2.69~+6.35] |
| 직진 |ω|<0.3 | 24 | +16.23 | -1.25 | +3.41 [+0.51~+4.10] |
| C 만 | 3507 | +0.81 | -1.90 | +2.70 [+0.28~+5.10] |
| A(PP) 만 | 1008 | +2.67 | +0.01 | +2.74 [+0.69~+4.54] |
| circles8 | 4508 | +1.15 | -1.40 | +2.72 [+0.35~+4.98] |
| 전체 | 4515 | +1.22 | -1.47 | +2.72 [+0.35~+4.98] |

bag 별 중앙값 (41 bag): 중앙값 +2.67°, 사분위 +2.47~+2.94°, 최소 +1.34, 최대 +3.49
속력 배율 |v_SLAM| / |v_바퀴| 중앙값 0.9887
