(define (problem robot-delivery-ma-prob)
  (:domain robot-delivery-ma)
  (:objects
    r1 r2 - robot
    warehouse zone_a zone_b - location
    pkg1 pkg2 - package
  )
  (:init
    (robot_at r1 warehouse)
    (robot_at r2 warehouse)
    (package_at pkg1 warehouse)
    (package_at pkg2 warehouse)
    (connected warehouse zone_a)
    (connected zone_a warehouse)
    (connected warehouse zone_b)
    (connected zone_b warehouse)
  )
  (:goal (and
    (delivered pkg1)
    (delivered pkg2)
  ))
)