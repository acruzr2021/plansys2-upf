(define (problem logistics-problem-1)
  (:domain logistics-simple)
  
  (:objects
    truck1 - vehicle
    loc-a loc-b loc-c loc-d - location
    pkg1 pkg2 pkg3 - package
  )

  (:init
    ;; Posiciones iniciales del vehículo
    (at-vehicle truck1 loc-a)
    (= (max-load truck1) 8)
    (= (max-load truck2) 8)
    (= (max-load truck3) 8)
    
    ;; Posiciones iniciales de los paquetes
    (at-package pkg1 loc-a)
    (at-package pkg2 loc-b)
    (at-package pkg3 loc-c)
    
    ;; Conexiones entre ubicaciones (bidireccionales)
    (connected loc-a loc-b)
    (connected loc-b loc-a)
    (connected loc-b loc-c)
    (connected loc-c loc-b)
    (connected loc-c loc-d)
    (connected loc-d loc-c)
    (connected loc-a loc-d)
    (connected loc-d loc-a)
  )
  
  (:goal
    (and
      ;; Queremos que todos los paquetes estén en loc-d
      (at-package pkg1 loc-d)
      (at-package pkg2 loc-d)
      (at-package pkg3 loc-d)
    )
  )
)