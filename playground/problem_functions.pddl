(define (problem test_problem)
  (:domain test_domain)
  
  (:objects
    room1 room2 - location
  )
  
  (:init
    ;; Diferentes formas de inicializar funciones
    
    ;; 1. Forma básica (valor directo)
    (temperature room1) 20.0
    
    ;; 2. Con asignación explícita
    (= (temperature room2) 25.0)

    ;; 3. Con increase (20.0 + 5.0 = 25.0)
    (increase (pressure room1) 5.0)
    
    ;; 4. Con decrease (valor_por_defecto - 3.0)
    (decrease (pressure room2) 3.0)
    
    ;; 5. Con scale-up (valor_por_defecto × 2.0)
    (scale-up (humidity room1) 2.0)
    
    ;; 6. Con scale-down (valor_por_defecto × 0.5)
    (scale-down (humidity room2) 0.5)
    
    ;; Predicados
    (at room1)
  )
  
  (:goal (at room2))
)