# DBlane flowchart

``` mermaid
graph TD;

    CalculatePoints[(Canditate Points)]:::style1 --> InitNewCluster[Init New cluster ]
    EmptyClusters[(Empty Clusters)]:::style1 --> InitNewCluster[Init New <br> cluster ]
    InitNewCluster:::style1 --> FOR{for}
    WHILE:::style2 --- question( Is next tail or head?) --->|True|END:::style5
    FOR:::style2 --> |number of clusters|WHILE{while}
    question:::style3 -->|False|ADD_Head/Tail[Add Head/Tail]:::style4 --> CalculateUnassigned[Calculate unassigned]
    CalculateUnassigned:::style4 --> FOR 
    classDef style1 fill:#34aec5,stroke:#152742,stroke-width:4px,color:#152742  
    classDef style2 fill:#152742,stroke:#34aec5,stroke-width:4px,color:#34aec5
    classDef style3 fill:#ffffff,stroke:#152742,stroke-width:4px,color:#152742
    classDef style4 fill:#f04444,stroke:#152742,stroke-width:4px,color:#152742
    classDef style5 fill:#88f088,stroke:#152742,stroke-width:4px,color:#152742
```

  