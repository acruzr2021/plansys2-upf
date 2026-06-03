(define (problem logistics-prob)
  (:domain logistics)
  (:objects
    pkg1 pkg2 - package
    truck - vehicle
    A B C D - location
  )
  (:init
    (at pkg1 A)
    (at pkg2 A)
    (vehicle-at truck A)
    (connected A B)
    (connected B C)
    (connected C D)
    (connected B A)
    (connected C B)
    (connected D C)
  )
  (:goal (and (at pkg1 D) (at pkg2 C)))
  (:metric minimize (total-cost))
)