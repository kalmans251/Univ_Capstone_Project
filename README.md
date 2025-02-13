캡스톤 디자인 프로젝트입니다.


##프로젝트 개요

세계적인 정세의 흐름을 살펴보면 최근 전쟁에 대한 불안감이 높아지는 경향이 상당부분 존재한다. 대한민국은 북한과의 휴전상태로 전쟁상태로 돌입한다하였을경우에 실효성 높은 드론을 제작하고자 함이 목표이다.
해당 조에는 박격포병을 수행한 경험이 있는 조원이 있으며, 박격포의 사격제원을 빠르고 정확하게 계산하는 능력의 중요성을 알아 드론과 사격제원 추출을 결합한 프로젝트를 수행하려고 한다.

물체의 자세는 쿼터니언으로 , 쿼터니언의 신뢰도를 높이는 과정은 칼만필터를 사용한다.
가속도 센서는 쿼터니언의 예측값(StateMent) 를 추정하고, 가속도센서와 지자기센서를 결합하여 관측값(MeasureMent)를 계산한다.

 예측값과 관측값의 공분산치를 사용하여 KalmansFilter를 적용하여 , 신뢰도가 높은 쿼터니언을 추출하며 쿼터니언과 칼만필터의 성능을 몸소 경험하고자 함이 목표이다.[신뢰성 높은 자세 쿼터니언 추출]

 다음으로는 위의 과정으로 추출한 쿼터니언을 통해 자세편차를 추출하여, 추출한 편차 쿼터니언을 PID제어를 이용하여 안정적인 비행을 구현하며 PID제어를 경험하고자 함이 다음 목표이다.[쿼터니언 편차 PID제어]
다음 과정으로는 자세 쿼터니언과 가속도 센서를 결합한 관성항법(INU)와 GPS 정보를 다시한번 KalmanFilter를 사용하여 신뢰도 높은 GPS값 추출이 목표이다. [StateMent:INU , MeasureMent:GPS 정보]
다음 과정으로는 GPS값을 통해 드론이 한 지점에 고정되어 호버링할수 있도록 PID제어를 구현함이 목표이다. [PID제어]
다음 과정으로는 안정적인 호버링을 수행하기 위해 관성항법(INU)의 수직성분과 기압센서의 KalmanFilter적용으로 안정적인 센서추출과 동시에 PID 호버링을 수행함이 목표이다. [StateMent:INU , MeasureMent:기압센서]

즉 전체적인 개요로는 세계적인 흐름을 보았을때 실효성이 높을만한 드론제작을 목표 ,쿼터니언 회전의 이해와 응용, KalmanFilter적용 과 응용력 , PID제어의 이해와 적용이 우리 프로젝트의 전체적인 개요이다.
