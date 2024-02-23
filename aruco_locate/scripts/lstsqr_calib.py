import numpy as np
from scipy import linalg

b = np.array([[0.9, 0, 0],
              [0.45,1.05,0],
              [0.75, 0.6, 0],
              [0.75, 1.05, 0],
              [1.2, 1.05, 0],
              [1.5, 1.05, 0],
              [1.5, 0.75, 0],
              [1.5, 0.3, 0],
              [0, 0.85, 0]])

# inverse
# A = np.array([[0.8623101962, -0.067624868, -0.2746952536],
#               [0.4167511544,1.012789115,-0.1489358065],
#               [0.7208727122,0.5574588293,-0.1895280865],
#               [0.7243381288,1.012310452,-0.1633824624],
#               [1.190174076,1.015501482,-0.1671963694],
#               [1.50022047,1.016471387,-0.166263543],
#               [1.491191279,0.7050533808,-0.1854114592],
#               [1.494170656,0.2466095346,-0.2293505477],
#               [-0.05575309326,0.8069195392,-0.1817071059]])

#raw
A = np.array([[-0.319879562,-0.7178722399,2.016335809],
              [0.1228893641,0.3844603358,1.987129126],
              [-0.1758036057,-0.08292779526,1.998265935],
              [-0.1862928708,0.3782067299,1.992688968],
              [-0.6500396591,0.369657223,1.999572736],
              [-0.9594974615,0.3645625467,1.995734349],
              [-0.9501708936,0.05289763974,1.988829396],
              [-0.9508784277,-0.4160417607,2.006438545],
              [0.5929896943,0.1913423816,1.992227733]])
p, *_ = linalg.lstsq(A, b)

D = A @ p
print(D)
# array([[ 0.96634906, -0.00256123,  0.        ],
#        [ 0.01068454,  0.99756876,  0.        ],
#        [-0.24550675, -0.25258789,  0.        ]])

# array([[-0.96768443, -0.02013843,  0.        ],
#        [-0.01115599,  0.96742494,  0.        ],
#        [ 0.28878065,  0.34058431,  0.        ]])

array([[-0.96768443, -0.02013843,  0.        ],
       [-0.01115599,  0.96742494,  0.        ],
       [ 0.28878065,  0.34058431,  0.        ]])