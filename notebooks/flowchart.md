# DBlane flowchart

``` mermaid
graph TD;

    CalculatePoints[(Canditate Points)]:::style1 --> InitNewCluster[Init New cluster ]
    EmptyClusters[(Empty Clusters)]:::style1 --> InitNewCluster[Init New <br> cluster ]
    InitNewCluster:::style1 --> FOR{for}
    WHILE:::dark --- question( Is next tail or head?) --->|True|END:::green
    FOR:::dark --> |number of clusters|WHILE{while}
    question:::white -->|False|ADD_Head/Tail[Add Head/Tail]:::red --> CalculateUnassigned[Calculate unassigned]
    CalculateUnassigned:::red --> FOR 
    classDef style1 fill:#34aec5,stroke:#152742,stroke-width:2px,color:#152742  
    classDef dark fill:#152742,stroke:#34aec5,stroke-width:2px,color:#34aec5
    classDef white fill:#ffffff,stroke:#152742,stroke-width:2px,color:#152742
    classDef red fill:#f04444,stroke:#152742,stroke-width:2px,color:#152742
    classDef green fill:#44ffaa,stroke:#152742,stroke-width:2px,color:#152742
```

  