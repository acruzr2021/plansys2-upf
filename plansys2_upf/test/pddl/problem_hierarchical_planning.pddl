(define (problem robot-delivery-htn-prob)
  (:domain robot-delivery-htn)
  (:objects
    r1 - robot
    warehouse zone_a zone_b - location
    pkg1 pkg2 - package
  )
  (:htn :parameters ()
    :ordered-subtasks (and
      (t1 (deliver_all))
    )
  )
  (:init
    (robot_at r1 warehouse)
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